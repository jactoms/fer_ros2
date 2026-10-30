"""
The simulation: a MuJoCo world with a scripted human hand, connected to ROS 2.

The hand (mocap body `body_name`) moves by `motion` (motions.py). Every body listed in
`commanded_bodies` (mocap bodies, default: tracker) is moved only by ROS commands, so other
nodes (fer_track_hand) can put their own objects into this one simulation without changing it.

Publishes, at publish_rate_hz:
  - /hand_mjc/position         geometry_msgs/PointStamped: hand position read back from MuJoCo
  - /hand_mjc/<body>/position  geometry_msgs/PointStamped: each commanded body
  - /hand_mjc/clock            rosgraph_msgs/Clock: simulation time (0 at start); not /clock,
                               so it does not clash with another simulation on the domain
  - /hand_mjc/marker           visualization_msgs/Marker (10 Hz): every body with its MuJoCo
                               shape, locked to its TF frame so RViz draws it where TF is
  - TF                         <frame_id> -> <body> for the hand and each commanded body
Subscribes:
  - /hand_mjc/target           geometry_msgs/PointStamped: moves the hand there; overrides
                               `motion` from then on
  - /hand_mjc/<body>/command   geometry_msgs/PointStamped: moves a commanded body there; it
                               stays at its scene position until the first command

Physics runs in real time (wall clock, mjc_sim.py). With viewer:=true the MuJoCo passive viewer
shows the scene; closing it leaves the simulation running.
"""
from __future__ import annotations

from fer_hand_mjc.mjc_sim import RealTimeSim, share_path
from fer_hand_mjc.motions import HandMotion
from fer_hand_mjc.node_main import run_node
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

MARKER_HZ = 10.0  # markers are frame-locked; they only need a slow refresh


class CommandedBody:
    """A mocap body moved by /hand_mjc/<name>/command."""

    def __init__(self, sim: RealTimeSim, node: Node, name: str) -> None:
        self.name = name
        self.body_id, self.mocap_id = sim.mocap_body(name)
        self.command = None
        self.position_pub = node.create_publisher(
            PointStamped, f'/hand_mjc/{name}/position', 10)
        node.create_subscription(
            PointStamped, f'/hand_mjc/{name}/command', self._on_command, 10)

    def _on_command(self, msg: PointStamped) -> None:
        self.command = [msg.point.x, msg.point.y, msg.point.z]


class HandMjc:
    """Steps the MuJoCo scene, moves the hand and the commanded bodies, publishes them."""

    def __init__(self, node: Node) -> None:
        self._node = node
        p = node.declare_parameter
        p('model_path', '')
        p('body_name', 'hand')
        p('commanded_bodies', ['tracker'])
        p('frame_id', 'world')
        p('publish_rate_hz', 100.0)
        p('viewer', True)
        p('motion', 'sine')
        p('start_position', [0.5, 0.0, 0.5])
        p('sine_amplitude', [0.1, 0.3, 0.15])
        p('sine_frequency', 0.08)
        p('random_range', [0.25, 0.35, 0.2])
        p('random_speed', 0.1)
        p('random_seed', 0)

        g = lambda name: node.get_parameter(name).value  # noqa: E731
        self._frame = g('frame_id')
        self._body_name = g('body_name')
        self._motion = HandMotion(
            motion=g('motion'),
            start=g('start_position'),
            sine_amplitude=g('sine_amplitude'),
            sine_frequency=g('sine_frequency'),
            random_range=g('random_range'),
            random_speed=g('random_speed'),
            random_seed=g('random_seed'),
        )
        rate_hz = g('publish_rate_hz')
        if rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')

        model_path = g('model_path') or share_path('models', 'hand_scene.xml')
        self._sim = RealTimeSim(model_path, self._set_mocap)
        self._body_id, self._mocap_id = self._sim.mocap_body(self._body_name)
        self._commanded = [
            CommandedBody(self._sim, node, name) for name in g('commanded_bodies') if name]
        self._target = None
        self._set_mocap()

        self._position_pub = node.create_publisher(PointStamped, '/hand_mjc/position', 10)
        self._clock_pub = node.create_publisher(Clock, '/hand_mjc/clock', 10)
        self._marker_pub = node.create_publisher(Marker, '/hand_mjc/marker', 10)
        self._tf = TransformBroadcaster(node)
        node.create_subscription(PointStamped, '/hand_mjc/target', self._on_target, 10)

        self._marker_every = max(1, round(rate_hz / MARKER_HZ))
        self._ticks = 0

        self._sim.start(g('viewer'), node.get_logger())
        node.create_timer(1.0 / rate_hz, self._tick)
        names = ', '.join(b.name for b in self._commanded) or 'none'
        node.get_logger().info(
            f"hand_mjc: {model_path}, body '{self._body_name}', "
            f"motion '{self._motion.motion}', commanded bodies: {names}, "
            f'{rate_hz:.0f} Hz on /hand_mjc/position')

    def close(self) -> None:
        """Close the viewer."""
        self._sim.close()

    def _on_target(self, msg: PointStamped) -> None:
        self._target = [msg.point.x, msg.point.y, msg.point.z]

    def _set_mocap(self) -> None:
        data = self._sim.data
        if self._target is not None:
            position = self._target
        else:
            position = self._motion.position(data.time, self._sim.model.opt.timestep)
        data.mocap_pos[self._mocap_id] = position
        for body in self._commanded:
            if body.command is not None:
                data.mocap_pos[body.mocap_id] = body.command

    def _tick(self) -> None:
        self._sim.step()
        self._publish()

    def _publish(self) -> None:
        time = self._sim.data.time
        clock = Clock()
        clock.clock.sec = int(time)
        clock.clock.nanosec = int((time % 1.0) * 1e9)
        self._clock_pub.publish(clock)

        stamp = self._node.get_clock().now().to_msg()
        self._ticks += 1
        self._publish_body(stamp, self._body_id, self._body_name, self._position_pub, 0)
        for i, body in enumerate(self._commanded, start=1):
            self._publish_body(stamp, body.body_id, body.name, body.position_pub, i)

    def _publish_body(self, stamp, body_id: int, name: str, pub, marker_id: int) -> None:
        x, y, z = self._sim.position(body_id)

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self._frame
        point.point.x, point.point.y, point.point.z = x, y, z
        pub.publish(point)

        tf = TransformStamped()
        tf.header = point.header
        tf.child_frame_id = name
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        self._tf.sendTransform(tf)

        if self._ticks % self._marker_every:
            return
        kind, extent, rgba = self._sim.shape(body_id)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = name  # at the body's TF frame, redrawn with every TF update
        marker.frame_locked = True
        marker.ns = 'hand_mjc'
        marker.id = marker_id
        marker.type = Marker.CUBE if kind == 'box' else Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = extent
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        self._marker_pub.publish(marker)


def main(args: list[str] | None = None) -> None:
    run_node('fer_hand_mjc', HandMjc, args)


if __name__ == '__main__':
    main()
