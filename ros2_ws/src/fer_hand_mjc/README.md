# fer_hand_mjc: the hand simulation

The MuJoCo simulation of the human hand for the handover: a scripted hand (a sphere) whose
position it publishes on ROS 2 and as TF, the black-box hand pose that `/motion/follow` reads.
It does not use the robot, `ros2_control` or the bringup. It is a normal ROS 2 node in the dev
container, so every node on the DDS domain sees its topics.

## Scene (`models/`)

| File | Content |
|---|---|
| `hand_mjc_world.xml` | World: physics options, view, floor, light, origin axes (0, 0, 0; x red, y green, z blue, 0.2 m) |
| `hand_scene.xml` | Includes the world; body `hand` (sphere, radius 0.04 m, skin colour) at (0.45, 0, 0.35) |

The hand is a mocap body: the node places it every physics step, no forces act on it, and it
does not collide. The positions published are read back from MuJoCo (`xpos`).

## Interfaces

| Interface | Type | Content |
|---|---|---|
| `/hand_mjc/position` | `geometry_msgs/PointStamped` (pub, 100 Hz) | Hand centre, `frame_id: world` |
| `/hand_mjc/clock` | `rosgraph_msgs/Clock` (pub, 100 Hz) | Simulation time, 0 at start. Not `/clock`, so it does not clash with the robot simulation |
| `/hand_mjc/marker` | `visualization_msgs/Marker` (pub, 10 Hz) | The hand sphere with its MuJoCo shape and colour, locked to the `hand` TF frame so RViz draws it exactly where TF is; shown in the robot RViz (`fer_moveit/rviz/moveit_conf.rviz`) and the hand RViz |
| TF `world → hand` | tf | The same position as a frame |
| TF `base → world` | tf_static | From the launch file (`parent_frame`, identity): links the hand to the robot's TF tree, so `/motion/follow` and the robot RViz see `hand`. On the real robot the tracker's calibrated camera pose takes this place |
| `/hand_mjc/target` | `geometry_msgs/PointStamped` (sub) | Moves the hand there; overrides `motion` from then on |

## Hand motions (`motion`)

| Motion | Movement (defaults) | Parameters |
|---|---|---|
| `static` | Holds `start_position` (0.45, 0, 0.35) | |
| `sine` | Oscillates around it: ±0.10 / ±0.25 / ±0.04 m in x / y / z at 0.08 Hz (12.5 s period, peak speed 0.14 m/s) | `sine_amplitude`, `sine_frequency` |
| `random` | Moves at 0.1 m/s to random waypoints in the box `random_center` ± `random_range`, a new one on arrival. `hand_mjc.yaml`: around the robot, x −0.45…0.55, y −0.55…0.55, z 0.31…0.39 m, but never through it: the hand centre stays more than `random_keepout_radius` (0.3 m) from the base axis, the straight paths between waypoints included | `random_center` (default `start_position`), `random_range`, `random_keepout_radius` (0: off), `random_speed`, `random_seed` (same seed, same path) |

`hand_speed:=<m/s>` on the launch file sets the speed without editing the yaml: `random_speed`
for `random`, the peak speed for `sine` (it sets `sine_frequency` from `sine_amplitude`).

The defaults are a handover height: with the gripper 10 cm above the hand (`/motion/follow` with
`offset: {position: {z: 0.1}}`), the gripper stays inside the Follow workspace of `fer_moveit`
(z ≤ 0.50 m in `base`); `test/test_motions.py` checks it. The `random` box of `hand_mjc.yaml` is
larger than that workspace (x 0.25…0.60, y ±0.30 m): while the hand is outside it, the gripper
holds.

All parameters are in `config/hand_mjc.yaml`.

## Code (`fer_hand_mjc/`)

| File | Content | Used by |
|---|---|---|
| `hand_mjc_node.py` | The node: parameters, hand, publishing | |
| `motions.py` | `HandMotion`: static, sine, random; no ROS, no MuJoCo | this node |
| `mjc_sim.py` | `RealTimeSim`: loads the scene, steps it in real time (skips ahead after a stall), optional viewer on the scene's camera, mocap body and shape lookup, clean close | this node |
| `node_main.py` | `run_node`: rclpy main with clean Ctrl+C handling; no MuJoCo | this node |

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

# with the robot running: MuJoCo viewer, sine, base -> world; the robot RViz shows the hand
ros2 launch fer_hand_mjc hand_mjc.launch.py
ros2 launch fer_hand_mjc hand_mjc.launch.py motion:=random     # static | sine | random
ros2 launch fer_hand_mjc hand_mjc.launch.py motion:=random hand_speed:=0.3   # m/s
# hand alone: no link to a robot, own RViz
ros2 launch fer_hand_mjc hand_mjc.launch.py parent_frame:='' rviz:=true

# second shell
ros2 topic echo /hand_mjc/position --field point
ros2 topic hz /hand_mjc/position
ros2 topic pub --once /hand_mjc/target geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 0.4, y: 0.2, z: 0.35}}"
```

Launch arguments: `motion`, `viewer`, `parent_frame` (default `base`; `''` for none), `rviz`
(default `false`), `params_file`, `rviz_config`, `log_level`.
Viewer: opens on the workspace (`<statistic>` in `hand_mjc_world.xml`) without its side panels
(Tab / Shift+Tab show them); left drag rotates, right drag pans, scroll zooms; closing it leaves
the simulation running. The hand RViz (`rviz/hand_mjc.rviz`, fixed frame `world`) shows the grid, the origin
axes, the TF frames and the hand.

## Tests

`colcon test --packages-select fer_hand_mjc` runs flake8, pep257 and `test/test_motions.py`
(the motions, without ROS).
