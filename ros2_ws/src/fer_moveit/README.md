# fer_moveit

MoveIt 2 configuration and launch files for the **Franka Emika Robot (FER)**,
targeting **ROS 2 Jazzy**.

This package is a downstream rework of the long-running
[`panda_moveit_config`](https://github.com/moveit/panda_moveit_config) package
(originally under the `ros-planning` GitHub organization). It has been adapted
to use the current `franka_description` URDF/SRDF, modern MoveIt 2 planning
pipelines (OMPL and Pilz with the new request/response adapters), and a launch entry
point with explicit arguments for controller selection, fake hardware, and
gripper loading.

It also holds the MoveIt motion server of the FER platform,
`fer_moveit_motion_server`, which serves the motion part of `fer_interfaces` through
`move_group`, and `/motion/follow` through MoveIt Servo (see [Motion server](#motion-server)
and [Follow](#follow-motionfollow)).

## Status

- Tested against **ROS 2 Jazzy** on Ubuntu 24.04.
- Tested with the **FER** (Franka Emika Robot / Panda).
  Other Franka models (e.g. FR3) have not been tested.

## Package layout

```
fer_moveit/
├── config/                  # MoveIt + controller YAMLs (kinematics, OMPL, joint/cartesian limits, …)
│                            # fer_moveit_motion_server.yaml: motion server parameters
├── launch/
│   ├── fer_move_group.launch.py            # move_group + optional RViz, with declared launch args
│   ├── fer_moveit_motion_server.launch.py  # motion server
│   ├── fer_streamer_check.launch.py        # check of the Cartesian streamer (Servo + Ruckig)
│   └── moveit_parameters.py                # robot model + Servo parameters for in-process MoveIt
├── include/fer_moveit/  # motion server: core/ (no ROS), adapters/, motion_server.hpp
├── src/                        # motion server sources, main.cpp, streamer_check.cpp
├── test/                       # gtests; move_group_test.launch.py runs against a real move_group
├── rviz/
│   └── moveit_conf.rviz     # RViz config tailored for this package
├── CMakeLists.txt
├── package.xml
├── LICENSE                  # BSD 3-Clause
├── NOTICE.md                # Upstream attribution + third-party file notice
└── README.md
```

## Dependencies

Declared in `package.xml`:

- `franka_description` (URDF/SRDF for FER)
- `xacro`
- `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`,
  `pilz_industrial_motion_planner`, `moveit_simple_controller_manager`
- motion server: `rclcpp`, `rclcpp_action`, `fer_interfaces`, `moveit_msgs`,
  `moveit_core`, `tf2_ros`, `tf2_geometry_msgs`; `moveit_servo` and `ruckig` for
  `/motion/follow`; `franka_msgs` for `hardware:=real` only (builds without it)

Install everything declared in the manifest with `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

From your colcon workspace root:

```bash
colcon build --symlink-install --packages-up-to fer_moveit
source install/setup.bash
```

## Launch

The launch file accepts the following arguments (defaults in parentheses):

| Argument               | Default        | Description |
| ---------------------- | -------------- | ----------- |
| `hardware`             | `mujoco`       | `real` or `mujoco` — selects `config/moveit_controllers_<hardware>.yaml` |
| `use_sim_time`         | `true`         | Use the simulated `/clock` |
| `use_rviz`             | `true`         | Start RViz with `rviz/moveit_conf.rviz` |
| `log_level`            | `warn`         | `debug` / `info` / `warn` / `error` / `fatal` |
| `db`                   | `False`        | Reserved for future warehouse DB use |
| `robot_ip`             | `""`           | Hostname/IP of the real robot |
| `namespace`            | `""`           | ROS namespace |
| `load_gripper`         | `true`         | Load the Franka Hand |
| `ee_id`                | `franka_hand`  | End-effector id: `none`, `franka_hand`, `cobot_pump` |
| `use_fake_hardware`    | `false`        | Use `ros2_control` fake hardware |
| `fake_sensor_commands` | `false`        | Fake sensor commands (only with `use_fake_hardware:=true`) |
| `arm_control_type`     | `effort`       | `effort`, `velocity` (real only) or `position` — selects `<type>_trajectory_controller` |
| `hand_control_type`    | `position`     | `effort` or `position` — selects `gripper_<type>_controller`; ignored for `real` |

`config/moveit_controllers_<hardware>.yaml` lists the controllers `move_group`
may route to on that hardware. The `arm_control_type` / `hand_control_type`
args pick the default ones: the `default` flag is set at launch time, so you do
not need to edit the YAML to switch controllers. On the real robot the gripper
is the `franka_gripper` node (`/fer_gripper/gripper_action`).

The controllers must be active in `ros2_control` before `move_group` can
execute; the bringup starts them inactive.

### Planning pipelines

| Pipeline | Planner | Used for |
|---|---|---|
| `ompl` (default) | OMPL, `RRTConnect` for `fer_arm` | collision-free paths; timed by `AddTimeOptimalParameterization` |
| `pilz_industrial_motion_planner` | Pilz `LIN` (also `PTP`, `CIRC`) | straight TCP lines; Pilz times them itself from `joint_limits.yaml` and `cartesian_limits.yaml` |

Both pipelines use the request adapters `ResolveConstraintFrames`,
`ValidateWorkspaceBounds`, `CheckStartStateBounds` and `CheckStartStateCollision`, and
check the result with `ValidateSolution`. Joint and Cartesian limits are loaded under
`robot_description_planning`. Trajectory execution is time-monitored: a motion may take
its planned duration × 1.1 + 0.5 s.

## Motion server

`fer_moveit_motion_server` serves the motion part of `fer_interfaces`. It is a client
of `move_group`: MoveIt plans and executes, the server adds validation, the world model,
`may_touch`, goal replacement and outcome codes. It never switches controllers.

| Served | Type |
|---|---|
| `/motion/move_to_pose` | action `fer_interfaces/MoveToPose` |
| `/motion/move_to_joints` | action `fer_interfaces/MoveToJoints` |
| `/motion/check_reachable` | service `fer_interfaces/CheckReachable` |
| `/motion/follow` | action `fer_interfaces/Follow` (see [Follow](#follow-motionfollow)) |

| Used | Purpose |
|---|---|
| `/world_model/query_objects` | FREE and GRASPED objects (fixed ones included) per request |
| `/apply_planning_scene`, `/get_planning_scene` | scene sync; allowed collisions for `may_touch` |
| `/check_state_validity` | contacts of a held object at the start of a straight path |
| `/move_action` | plan only |
| `/execute_trajectory` | execution on `move_group`'s default arm controller (`arm_control_type`) |
| `/trajectory_execution_event` | `"stop"` on cancel or replacement |
| `/joint_states`, TF | wait for rest; goal pose → `base` |
| `/franka_robot_state_broadcaster/robot_state` | robot mode (`hardware:=real`) |
| `/effort_trajectory_controller/joint_trajectory` | Follow: one short joint trajectory every 10 ms |
| `/monitored_planning_scene` | Follow: Servo's collision check |

From goal to motion:

1. **Validate:** `speed_scaling` in (0, 1], pose frame known to TF (converted to `base`
   once, with the latest transform) → else `INVALID_GOAL`. Real: robot in reflex or user
   stop → `ROBOT_ERROR`.
2. **Scene:** `QueryObjects`, then one `ApplyPlanningScene`: FREE and fixed objects as
   boxes, GRASPED objects attached to `held_by` with the hand links as touch links,
   objects gone from the world model removed. Unknown `may_touch` id → `NOT_FOUND`; no
   answer → `TIMEOUT`.
3. **`may_touch`:** the current allowed-collision matrix plus (object, hand link) pairs,
   sent with the plan request only. A diff replaces the whole matrix, hence the full copy.
   **Held object at the start:** on a `PATH_STRAIGHT` motion while an object is held,
   `move_group` is asked which world objects the held one touches at the start state, and
   those pairs are allowed too, so a box lifts off the table it rests on. The pairs hold
   for the whole straight path; free paths and joint targets get none and fail on such a
   start state.
4. **Plan:** plan-only `MoveGroup` on `fer_arm` with velocity and acceleration scaling =
   `speed_scaling`: `PATH_FREE` and joint targets with `ompl`, `PATH_STRAIGHT` with Pilz
   `LIN`. A planning failure → `NO_PATH`, the reason in `message`: `move_group`'s plan-only
   path reports every planner error as `FAILURE`, so `UNREACHABLE` does not occur.
5. **Execute:** `ExecuteTrajectory` with no controller named, so `move_group` uses the arm
   controller selected by `arm_control_type`. It checks the start state, sends the
   trajectory to the controller and watches the time.
   Failure → `EXECUTION_FAILED` (`ROBOT_ERROR` if the robot is in reflex or user stop).
6. **Cancel or replacement:** the server publishes `"stop"`, the controller decelerates
   (`decelerate_on_cancel`), and the goal ends `CANCELLED` once every joint is below
   `rest_velocity`. The next goal plans from there. A cancel during planning takes effect
   when `move_group` has finished planning.

`CheckReachable` plans each target from the previous target's end configuration and
never executes.

### Follow (`/motion/follow`)

Keeps the TCP at a fixed offset from a moving TF frame (e.g. a tracked hand) until the client
cancels. No planning: every 10 ms the server computes a TCP velocity, MoveIt Servo (run as a
library in the server's process, `adapters/servo_streamer`) turns it into a joint step, Ruckig
(`core/joint_shaper`) shapes it within velocity, acceleration and jerk limits, and the result
goes to the arm's `effort_trajectory_controller` as a short joint trajectory. The same
controller as for planned motions: no controller switch, and the same on the real FER and in
MuJoCo.

**Goal** (`fer_interfaces/Follow`):

| Field | Default | Meaning |
|---|---|---|
| `target_frame` | – | TF frame to follow, read again every cycle |
| `offset` | identity | TCP position in `target_frame` (e.g. z 0.1 = 10 cm above the hand); its orientation is ignored for now |
| `follow_orientation` | false | true → `UNSUPPORTED` (position only for now); the TCP keeps the orientation it had at goal start |
| `speed_scaling` | 1.0 | (0, 1]; scales `follow_max_speed` and `follow_max_rotation_speed`. Below 1 the TCP can fall behind a moving hand |
| `target_timeout` | 0.5 s | target not updated this long → `NOT_FOUND` |

**Feedback:** `distance`, TCP to its goal position (m), every `follow_feedback_period`.

**Each cycle:**

1. Look up the target in `base`. Its own stamps decide whether a sample is new (the target may
   run on another clock); stamp 0 = a static frame, always current.
2. Goal = target × offset, with the start orientation. **Workspace:** goal outside the box
   `follow_workspace_min/max` (away from the arm's singular regions) → the TCP holds the last
   goal that was inside ("goal outside the workspace, holding") and follows again once it is
   back inside.
3. TCP velocity v = v_target + `follow_gain` × (goal − TCP), capped at `follow_max_speed` ×
   `speed_scaling` (`core/follow_law`). The target velocity v_target (finite difference,
   low-pass `follow_velocity_time_constant`) is fed forward only while the target is fresh:
   in full for `follow_velocity_hold`, then faded to 0 over `follow_velocity_fade`. A tracker
   that stops sending does not carry the TCP on.
4. Servo: twist → joint step, with its collision check and singularity slow-down / stop
   (`moveit_servo.lower_singularity_threshold`, `hard_stop_singularity_threshold`).
5. **Posture term** (`follow_posture_gain` > 0): a null-space joint velocity
   −k·(I − J⁺J)·(q − `follow_posture_rest`) is added. It does not move the TCP. It holds the
   elbow of the 7-joint arm, which Servo leaves free: without it the elbow drifts a little with
   every cycle of a periodic hand motion until Servo slows down near a singularity (sine hand:
   after ~3 min). Not applied while Servo reports a halt.
6. **Joint shaping** (`follow_shaping_profile`): Ruckig, velocity mode, all joints
   time-synchronised; the 10 ms command never exceeds `follow_shaping.<profile>` (velocity,
   acceleration, jerk per joint). `off` sends Servo's step unshaped (steps in acceleration,
   torque-rate spikes).
7. Publish the trajectory (stamped on a fixed 10 ms grid); log each Servo status change
   ("follow: ...").

**Outcomes** (on every end except a start failure the arm is braked to rest first, within the
shaping limits; if it does not come to rest within `rest_timeout`: `EXECUTION_FAILED`):

| Outcome | When |
|---|---|
| `CANCELLED` | the normal end: cancelled by the client, or replaced by a newer goal on any `/motion/*` action |
| `INVALID_GOAL` | `speed_scaling` outside (0, 1], empty `target_frame`, `target_timeout` ≤ 0 |
| `UNSUPPORTED` | `follow_orientation` true; `hardware:=mock` (no effort controller, no streamer) |
| `NOT_FOUND` | target frame not in TF at the start, lost, or not updated for `target_timeout` |
| `UNREACHABLE` | **halt watchdog:** Servo reports a halt (collision, singularity, joint bound) for `follow_halt_timeout`; **stall watchdog:** over `follow_stall_window`, the TCP moves along the commanded direction at less than `follow_stall_ratio` of the commanded speed for `follow_stall_timeout` (Servo can slow down towards a singularity without ever reporting a halt). Commands below `follow_stall_min_speed` are not judged |
| `EXECUTION_FAILED` | Servo returned an invalid state; nobody listens on the controller topic (within 2 s); the arm did not come to rest |
| `TIMEOUT` | no current joint state at the start |
| `ROBOT_ERROR` | real robot in reflex or user stop |

A client recovers from `UNREACHABLE` with a planned move (e.g. `MoveToJoints` to the ready
pose). Before each handover the arm should start from the same pose anyway.

**Shaping profiles** in `config/fer_moveit_motion_server.yaml`:

| Profile | Velocity | Acceleration | Jerk | Use |
|---|---|---|---|---|
| `conservative` (default) | `joint_limits.yaml` | `joint_limits.yaml` (datasheet / 4) | 10 × acceleration | the real FER, all experiments |
| `max_sim` | datasheet | datasheet | datasheet | MuJoCo only, fastest response |
| `off` | – | – | – | comparison only |

MuJoCo, 10 cm target jumps (20 per run, 22 Nov): `conservative` t90 0.8 s, torque rate per
jump median ~250 Nm/s (max 770), 0/20 jumps over 1000 Nm/s; `off` t90 0.73 s, median ~2300
Nm/s, 18–19/20 over. Sine hand: median tracking error 6 mm.

#### Check of the streamer

Moves the TCP a short distance along one axis of `base` through Servo and Ruckig and stops.
Passes if the TCP covers 80–120 % of the distance and drifts less than 1 cm on the other axes.
Needs the stack with `effort_trajectory_controller` active; the motion server must not stream
at the same time.

```bash
ros2 launch fer_moveit fer_streamer_check.launch.py axis:=x distance:=0.05 speed:=0.05
```

| Argument | Default | |
|---|---|---|
| `axis` | `x` | `x`, `y` or `z` |
| `distance` | `0.05` | m, signed |
| `speed` | `0.05` | m/s, > 0 |

### Launch

```bash
ros2 launch fer_moveit fer_moveit_motion_server.launch.py hardware:=mujoco
```

| Argument | Default | Description |
|---|---|---|
| `hardware` | `mujoco` | `real`, `mujoco` or `mock`; `use_sim_time` follows it. `mock` needs `arm_control_type:=position hand_control_type:=position` in the bringup; Follow is `UNSUPPORTED` there |
| `log_level` | `info` | node log level |
| `params_file` | `''` | `''` selects `config/fer_moveit_motion_server.yaml` |

`move_group` must run (`fer_move_group.launch.py`) and its arm controller must be active.
For Follow the launch file also passes the robot model (`moveit_parameters.py`) and the
`moveit_servo` block to the node: Servo runs in its process.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `planning_time`, `planning_attempts` | 5.0 s, 1 | per plan |
| `goal_position_tolerance`, `goal_orientation_tolerance` | 0.001 m, 0.01 rad | pose targets |
| `rest_velocity`, `rest_timeout` | 0.01 rad/s, 3.0 s | at rest after a stop |
| `tf_timeout`, `world_model_timeout`, `move_group_timeout`, `startup_timeout` | 0.2, 2.0, 7.0, 10.0 s | |

Follow (`/motion/follow`):

| Parameter | Default | Meaning |
|---|---|---|
| `follow_gain`, `follow_max_speed` | 2.0 1/s, 0.25 m/s | TCP velocity = feedforward + gain × error, capped (× `speed_scaling`) |
| `follow_rotation_gain`, `follow_max_rotation_speed` | 2.0 1/s, 0.5 rad/s | the same for the orientation (holds the start orientation) |
| `follow_velocity_time_constant` | 0.05 s | low-pass of the target velocity estimate |
| `follow_velocity_hold`, `follow_velocity_fade` | 0.05 s, 0.05 s | feedforward in full while the newest target sample is this old, then faded to 0 |
| `follow_feedback_period` | 0.1 s | feedback rate |
| `follow_workspace_min`, `follow_workspace_max` | [0.25, −0.30, 0.25], [0.60, 0.30, 0.50] m | box in `base` the goal is kept in |
| `follow_halt_timeout` | 1.0 s | halt watchdog |
| `follow_stall_window`, `follow_stall_min_speed`, `follow_stall_ratio`, `follow_stall_timeout` | 0.5 s, 0.03 m/s, 0.2, 1.5 s | stall watchdog |
| `follow_shaping_profile` | `conservative` | profile name, or `off` |
| `follow_shaping.<profile>.max_velocity`, `.max_acceleration`, `.max_jerk` | see the YAML | 7 values each, `fer_joint1..7` |
| `follow_posture_gain` | 1.0 1/s | posture term; 0 = off |
| `follow_posture_max_speed` | 0.2 rad/s | cap of the posture velocity (norm) |
| `follow_posture_rest` | the ready pose | 7 values, rad |
| `moveit_servo.*` | see the YAML | Servo, run as a library. On the FER keep `publish_joint_velocities: false` (else the JTC rejects every message) and `use_smoothing: false` (else the TCP covers only ~57 % of the commanded motion); `command_out_topic` = the effort JTC; singularity thresholds 17 / 30 |

Fixed in the code, since they change only with the robot or the planner choice: group
`fer_arm`, TCP `fer_hand_tcp`, hand links, arm joints, frame `base`, pipelines `ompl` and
Pilz `LIN`, topic `/franka_robot_state_broadcaster/robot_state` (remap it in the launch
file if needed).

### Tests

```bash
colcon test --packages-select fer_moveit
colcon test-result --verbose
```

`test_checks` covers the core; `test_follow_law` the follow law, the watchdogs and the
posture term; `test_joint_shaper` the Ruckig shaping (limits never exceeded, synchronised
joints, braking); `test_contract` runs the server against fake `move_group`, world model and
arm; `test_contract_follow` runs Follow against a fake streamer and TF (outcomes, workspace,
watchdogs, feedforward, replacement); `move_group_test.launch.py` starts the real
`move_group` with this configuration and checks straight paths, `may_touch`, held objects,
speed scaling, the closed-finger start state and the stop. Every test runs in its own DDS
domain on localhost.

### Known limits

- A cancel during planning takes effect once `move_group` has finished planning.
- `"stop"` stops every execution in `move_group`, not only this server's.
- Objects this server wrote stay in `move_group`'s scene; after a server restart they are
  not removed until an object with the same id is written again.
- Poses are converted with the latest TF (arm at rest).
- Planning failures are always `NO_PATH`, never `UNREACHABLE` (see step 4).
- Straight lines depend on the pose: Pilz checks them against `joint_limits.yaml`, and
  `cartesian_limits.yaml` is sized for poses near `ready`. Close to a singularity even a
  slow straight line can end `NO_PATH`.
- Follow tracks the position only; the orientation stays as at goal start.
- Follow's workspace box is fixed by parameters, not derived from the robot's reach.
- In MuJoCo the measured joint state reaches the controller in bursts, which shows up as
  torque-rate peaks that the real robot would not have; judge torque rates on the FER.

Example:

```bash
ros2 launch fer_moveit fer_move_group.launch.py \
    hardware:=mujoco \
    use_sim_time:=true \
    arm_control_type:=effort \
    hand_control_type:=position
```

## License

This package is released under the **BSD 3-Clause License** — see
[`LICENSE`](LICENSE). The motion server (`include/`, `src/`, `test/`,
`launch/fer_moveit_motion_server.launch.py`, `launch/fer_streamer_check.launch.py`,
`launch/moveit_parameters.py`) is released under the **Apache License, Version 2.0**.

`launch/fer_move_group.launch.py` is adapted from a Franka Robotics example
distributed under the **Apache License, Version 2.0**, and retains its
original header. See [`NOTICE.md`](NOTICE.md) for upstream attribution and
third-party file notices.

## Contributing

See `CONTRIBUTING.md` at the repository root.
