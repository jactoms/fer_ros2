"""
Hand tracker: moves the tracker cube of the fer_hand_mjc simulation to the hand.

A pure controller, no simulation of its own: it reads the hand and tracker positions from the
simulation and sends the tracker commands. start_delay seconds of simulation time after this
node starts, it drives the cube to sit side by side with the hand and to move along with it
(core/tracker.py). Before that the cube stays where it is.

Subscribes:
  - /hand_mjc/clock              rosgraph_msgs/Clock: simulation time
  - /hand_mjc/position           geometry_msgs/PointStamped: hand centre
  - /hand_mjc/<tracker>/position geometry_msgs/PointStamped: tracker centre (measured)
Publishes:
  - /hand_mjc/<tracker>/command  geometry_msgs/PointStamped: tracker target, per hand sample
  - /track_hand/distance         std_msgs/Float64: hand to tracker centre distance (m)
  - /track_hand/state            std_msgs/String: WAITING | TRACKING | SUCCESS (latched)
  - /track_hand/marker           visualization_msgs/Marker (10 Hz): state label above the hand,
                                 locked to the hand's TF frame
"""
from __future__ import annotations

from fer_hand_mjc.node_main import run_node
from fer_track_hand.core.tracker import (
    distance, HandTracker, SUCCESS, TrackerConfig, TRACKING, WAITING)
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

STATE_RGB = {WAITING: (0.7, 0.7, 0.7), TRACKING: (1.0, 0.8, 0.1), SUCCESS: (0.1, 0.9, 0.2)}
LABEL_EVERY = 10  # hand samples per label update (10 Hz); the label is frame-locked


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _xyz(msg: PointStamped) -> tuple[float, float, float]:
    return (msg.point.x, msg.point.y, msg.point.z)


class TrackHand:
    """Runs HandTracker on the hand samples of the simulation."""

    def __init__(self, node: Node) -> None:
        self._node = node
        p = node.declare_parameter
        p('frame_id', 'world')
        p('tracker_body', 'tracker')
        p('start_delay', 4.0)
        p('gap', 0.08)
        p('side', 'auto')
        p('gain', 2.0)
        p('max_speed', 0.25)
        p('velocity_time_constant', 0.05)
        p('tolerance', 0.01)
        p('hold_time', 2.0)

        g = lambda name: node.get_parameter(name).value  # noqa: E731
        self._frame = g('frame_id')
        self._start_delay = g('start_delay')
        self._tracker = HandTracker(TrackerConfig(
            gap=g('gap'),
            side=g('side'),
            gain=g('gain'),
            max_speed=g('max_speed'),
            velocity_time_constant=g('velocity_time_constant'),
            tolerance=g('tolerance'),
            hold_time=g('hold_time'),
        ))
        body = g('tracker_body')
        self._sim_time = None
        self._start_at = None   # simulation time when tracking starts
        self._tracker_pos = None
        self._t_prev = None
        self._state = None
        self._samples = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._command_pub = node.create_publisher(
            PointStamped, f'/hand_mjc/{body}/command', 10)
        self._distance_pub = node.create_publisher(Float64, '/track_hand/distance', 10)
        self._state_pub = node.create_publisher(String, '/track_hand/state', latched)
        self._marker_pub = node.create_publisher(Marker, '/track_hand/marker', 10)
        node.create_subscription(Clock, '/hand_mjc/clock', self._on_clock, 10)
        node.create_subscription(
            PointStamped, f'/hand_mjc/{body}/position', self._on_tracker, 10)
        node.create_subscription(PointStamped, '/hand_mjc/position', self._on_hand, 10)

        cfg = self._tracker.cfg
        node.get_logger().info(
            f"track_hand: body '{body}', tracking {self._start_delay:.1f} s (sim time) after "
            f'start, success when the centre distance is {cfg.gap:.3f} +- {cfg.tolerance:.3f} m '
            f'for {cfg.hold_time:.1f} s, max speed {cfg.max_speed:.2f} m/s')

    def _on_clock(self, msg: Clock) -> None:
        self._sim_time = _seconds(msg.clock)
        if self._start_at is None:  # first contact with the simulation
            self._start_at = self._sim_time + self._start_delay

    def _on_tracker(self, msg: PointStamped) -> None:
        self._tracker_pos = _xyz(msg)

    def _on_hand(self, msg: PointStamped) -> None:
        hand = _xyz(msg)
        t = _seconds(msg.header.stamp)
        dt = 0.0 if self._t_prev is None else t - self._t_prev
        self._t_prev = t
        self._tracker.observe_hand(hand, dt)
        if self._start_at is None or self._tracker_pos is None:
            return  # simulation not seen yet

        if self._sim_time < self._start_at:
            d = distance(hand, self._tracker_pos)
            self._set_state(WAITING, d)
        else:
            self._publish_command(self._tracker.step(t, dt, hand, self._tracker_pos))
            d = self._tracker.distance
            self._set_state(self._tracker.state, d)
        self._distance_pub.publish(Float64(data=d))
        self._samples += 1
        if self._samples % LABEL_EVERY == 1:
            self._publish_label(msg, d)

    def _set_state(self, state: str, d: float) -> None:
        if state == self._state:
            return
        self._state = state
        self._state_pub.publish(String(data=state))
        self._node.get_logger().info(
            f'{state} at sim t = {self._sim_time:.2f} s, distance {d:.3f} m')

    def _publish_command(self, command) -> None:
        msg = PointStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.point.x, msg.point.y, msg.point.z = command
        self._command_pub.publish(msg)

    def _publish_label(self, hand: PointStamped, d: float) -> None:
        marker = Marker()
        marker.header.stamp = hand.header.stamp
        marker.header.frame_id = 'hand'  # fer_hand_mjc's TF frame of the hand
        marker.frame_locked = True
        marker.ns = 'track_hand'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.04
        marker.color.r, marker.color.g, marker.color.b = STATE_RGB[self._state]
        marker.color.a = 1.0
        # No spaces: RViz's text rendering spreads words far apart.
        marker.text = f'{self._state}\n{100.0 * d:.1f}cm'
        self._marker_pub.publish(marker)


def main(args: list[str] | None = None) -> None:
    run_node('fer_track_hand', TrackHand, args)


if __name__ == '__main__':
    main()
