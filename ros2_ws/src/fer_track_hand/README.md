# fer_track_hand: the hand tracker

A controller that moves the tracker cube of the `fer_hand_mjc` simulation side by side with
the hand and keeps it there while the hand moves. It has no simulation of its own: it reads
positions from the simulation and sends commands to it, like a controller of a real robot
would. Start it whenever you want while the simulation runs.

## Behaviour

| State | When |
|---|---|
| `WAITING` | Less than `start_delay` (4 s) of simulation time since this node started. The cube stays where it is. |
| `TRACKING` | The cube moves to the side-by-side position: face to face with the hand, same height, centre distance `gap` = hand radius + cube half size = 2 × 0.04 m = 0.08 m. The side (±x or ±y of the hand) is fixed when tracking starts: the side the cube comes from (`side: auto`), or the one configured. |
| `SUCCESS` | The centre distance has stayed within 0.08 ± `tolerance` (0.01 m) for `hold_time` (2 s): side by side and moving along with the hand. Leaving the band goes back to `TRACKING`. |

Control (`fer_track_hand/core/tracker.py`, no ROS): command velocity = estimated hand velocity
(feedforward, so the cube keeps up instead of trailing) + `gain` (2 /s) × offset to the
side-by-side position, limited to `max_speed` (0.25 m/s; the hand moves at most 0.18 m/s).

## Interfaces

| Interface | Type | Content |
|---|---|---|
| `/hand_mjc/clock` | `rosgraph_msgs/Clock` (sub) | Simulation time |
| `/hand_mjc/position` | `geometry_msgs/PointStamped` (sub) | Hand centre; every sample runs one control step |
| `/hand_mjc/tracker/position` | `geometry_msgs/PointStamped` (sub) | Cube centre, measured in the simulation |
| `/hand_mjc/tracker/command` | `geometry_msgs/PointStamped` (pub, 100 Hz while tracking) | Cube target |
| `/track_hand/distance` | `std_msgs/Float64` (pub) | Hand to cube centre distance (m) |
| `/track_hand/state` | `std_msgs/String` (pub, latched) | `WAITING`, `TRACKING` or `SUCCESS`, on change |
| `/track_hand/marker` | `visualization_msgs/Marker` (pub, 10 Hz) | State label above the hand, e.g. `SUCCESS` / `7.9cm`, locked to the `hand` TF frame (shown in the `fer_hand_mjc` RViz) |

The node logs each state change with the simulation time. Parameters are in
`config/track_hand.yaml`; `tracker_body` names the commanded body in `fer_hand_mjc`.

## Run (in the dev container)

```bash
cd ~/ros2_ws
colcon build --symlink-install --merge-install --packages-select fer_hand_mjc fer_track_hand
source install/setup.bash

# shell 1: the simulation (MuJoCo window + RViz)
ros2 launch fer_hand_mjc hand_mjc.launch.py motion:=random      # static | sine | random

# shell 2: the tracker, whenever you want; it starts moving the cube 4 s later
ros2 launch fer_track_hand track_hand.launch.py

# shell 3
ros2 topic echo /track_hand/state
ros2 topic echo /track_hand/distance
```

Stopping the tracker (Ctrl+C) leaves the cube where it is; started again, it tracks from there.
Both from one command: `ros2 launch fer_track_hand track_hand.launch.py hand:=true motion:=sine`
(the launch file then includes `fer_hand_mjc`'s launch file and forwards `motion`, `viewer`,
`rviz`, `rviz_config`, `hand_params_file`, `log_level`).

## Tests

`colcon test --packages-select fer_track_hand` runs flake8, pep257 and `test/test_tracker.py`:
the tracker in closed loop against `fer_hand_mjc`'s `HandMotion` for static, sine and random
(5 seeds): SUCCESS within 8 s, in the band all the time after it, never overlapping the hand by
more than the tolerance, the cube staying on its side, speed limit, hold time, side choice.
