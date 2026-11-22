"""
Robot model parameters for a node that runs MoveIt in its own process (MoveIt Servo).

The same FER description, kinematics and limits as fer_move_group.launch.py gives move_group,
so the in-process model and move_group's model are identical. Not a launch file: imported by
fer_moveit_motion_server.launch.py and fer_streamer_check.launch.py.
"""
import os
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def _yaml(path: str) -> Dict:
    with open(os.path.join(get_package_share_directory('fer_moveit'), path)) as f:
        return yaml.safe_load(f)


def moveit_parameters(load_gripper: str = 'true', ee_id: str = 'franka_hand') -> List[Dict]:
    """Return robot_description, _semantic, _kinematics and _planning as node parameters."""
    robot = os.path.join(get_package_share_directory('franka_description'), 'robots', 'fer')
    urdf = xacro.process_file(
        os.path.join(robot, 'fer.urdf.xacro'),
        mappings={'hand': load_gripper, 'ee_id': ee_id, 'ros2_control': 'false'}).toxml()
    srdf = xacro.process_file(
        os.path.join(robot, 'fer.srdf.xacro'),
        mappings={'hand': load_gripper, 'ee_id': ee_id}).toxml()
    return [
        {'robot_description': ParameterValue(urdf, value_type=str)},
        {'robot_description_semantic': ParameterValue(srdf, value_type=str)},
        {'robot_description_kinematics': _yaml('config/kinematics.yaml')},
        {'robot_description_planning': {**_yaml('config/joint_limits.yaml'),
                                        **_yaml('config/cartesian_limits.yaml')}},
    ]


def servo_parameters(params_file: str) -> Dict:
    """
    Return what ServoStreamer reads from a motion server parameters file.

    The moveit_servo block and the joint shaping (follow_shaping_profile, follow_shaping).
    """
    with open(params_file) as f:
        params = yaml.safe_load(f)['fer_moveit_motion_server']['ros__parameters']
    names = ('moveit_servo', 'follow_shaping_profile', 'follow_shaping')
    return {name: params[name] for name in names if name in params}
