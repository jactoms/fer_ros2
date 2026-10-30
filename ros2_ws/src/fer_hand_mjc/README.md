# fer_hand_mjc: the hand simulation

The one MuJoCo simulation of the hand-tracking setup. It holds a scripted human hand (a
sphere) and bodies that other nodes move over ROS (the tracker cube of `fer_track_hand`), and
publishes all their positions on ROS 2. It does not use the robot, `ros2_control` or the
bringup. It is a normal ROS 2 node in the dev container, so every node on the DDS domain sees
its topics.

## Scene (`models/`)

| File | Content |
|---|---|
| `hand_mjc_world.xml` | World: physics options, view, floor, light, origin axes (0, 0, 0; x red, y green, z blue, 0.2 m) |
| `hand_scene.xml` | Includes the world; body `hand` (sphere, radius 0.04 m, skin colour) at (0.5, 0, 0.5); body `tracker` (cube, half size 0.04 m, blue) at (0.2, -0.6, 0.2) |

Both bodies are mocap bodies: the node places them every physics step, no forces act on them,
and they do not collide. The positions published are read back from MuJoCo (`xpos`).

## Interfaces

| Interface | Type | Content |
|---|---|---|
| `/hand_mjc/position` | `geometry_msgs/PointStamped` (pub, 100 Hz) | Hand centre, `frame_id: world` |
| `/hand_mjc/<body>/position` | `geometry_msgs/PointStamped` (pub, 100 Hz) | Centre of each commanded body (`/hand_mjc/tracker/position`) |
| `/hand_mjc/clock` | `rosgraph_msgs/Clock` (pub, 100 Hz) | Simulation time, 0 at start. Not `/clock`, so it does not clash with the robot simulation |
| `/hand_mjc/marker` | `visualization_msgs/Marker` (pub, 10 Hz) | Every body with its MuJoCo shape and colour (hand sphere id 0, tracker cube id 1), locked to the body's TF frame so RViz draws it exactly where TF is |
| TF `world → hand`, `world → <body>` | tf | The same positions as frames |
| `/hand_mjc/target` | `geometry_msgs/PointStamped` (sub) | Moves the hand there; overrides `motion` from then on |
| `/hand_mjc/<body>/command` | `geometry_msgs/PointStamped` (sub) | Moves a commanded body there (`/hand_mjc/tracker/command`); it stays at its scene position until the first command |

## Hand motions (`motion`)

| Motion | Movement (defaults) | Parameters |
|---|---|---|
| `static` | Holds `start_position` (0.5, 0, 0.5) | |
| `sine` | Oscillates around it: ±0.10 / ±0.30 / ±0.15 m in x / y / z at 0.08 Hz (12.5 s period, peak speed 0.18 m/s) | `sine_amplitude`, `sine_frequency` |
| `random` | Moves at 0.1 m/s to random waypoints in the box `start_position` ± (0.25, 0.35, 0.2) m, a new one on arrival | `random_range`, `random_speed`, `random_seed` (same seed, same path) |

All parameters are in `config/hand_mjc.yaml`. `commanded_bodies` lists the bodies moved over
ROS (default `[tracker]`).

## Code (`fer_hand_mjc/`)

| File | Content | Used by |
|---|---|---|
| `hand_mjc_node.py` | The node: parameters, hand and commanded bodies, publishing | |
| `motions.py` | `HandMotion`: static, sine, random; no ROS, no MuJoCo | this node; `fer_track_hand` tests |
| `mjc_sim.py` | `RealTimeSim`: loads the scene, steps it in real time (skips ahead after a stall), optional viewer on the scene's camera, mocap body and shape lookup, clean close | this node |
| `node_main.py` | `run_node`: rclpy main with clean Ctrl+C handling; no MuJoCo | this node; `fer_track_hand` |

## Requirements

MuJoCo Python bindings (`pip mujoco==3.4.0`, the version of `mujoco_vendor`'s library).
`docker/Dockerfile` installs them when `WITH_MUJOCO=1` (the x86_64 default). In a container
from an older image, install them once:

```bash
pip3 install --user --break-system-packages mujoco==3.4.0
```

## Build and run (in the dev container)

```bash
cd ~/ros2_ws
colcon build --symlink-install --merge-install --packages-select fer_hand_mjc
source install/setup.bash

ros2 launch fer_hand_mjc hand_mjc.launch.py                    # MuJoCo viewer + RViz, sine
ros2 launch fer_hand_mjc hand_mjc.launch.py motion:=random     # static | sine | random
ros2 launch fer_hand_mjc hand_mjc.launch.py viewer:=false rviz:=false

# second shell
ros2 topic echo /hand_mjc/position --field point
ros2 topic hz /hand_mjc/position
ros2 topic pub --once /hand_mjc/target geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 0.4, y: 0.2, z: 0.6}}"
ros2 topic pub --once /hand_mjc/tracker/command geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 0.5, y: -0.3, z: 0.4}}"
```

Launch arguments: `motion`, `viewer`, `rviz`, `params_file`, `rviz_config`, `log_level`.
Viewer: opens on the workspace (`<statistic>` in `hand_mjc_world.xml`) without its side panels
(Tab / Shift+Tab show them); left drag rotates, right drag pans, scroll zooms; closing it leaves
the simulation running. RViz (`rviz/hand_mjc.rviz`) shows the grid, the origin axes, the TF frames, the
bodies and, when `fer_track_hand` runs, its state label.

## Tests

`colcon test --packages-select fer_hand_mjc` runs flake8, pep257 and `test/test_motions.py`
(the motions, without ROS).
