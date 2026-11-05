"""MoveIt motion server of the FER platform; move_group comes from fer_move_group.launch.py."""
import os
import sys
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.join(get_package_share_directory('fer_moveit'), 'launch'))
from moveit_parameters import moveit_parameters  # noqa: E402

HARDWARE = ('real', 'mujoco')


def launch_setup(context, *args, **kwargs) -> List[Node]:
    hardware = LaunchConfiguration('hardware').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context) or PathJoinSubstitution(
        [FindPackageShare('fer_moveit'), 'config', 'fer_moveit_motion_server.yaml']
    ).perform(context)

    return [
        Node(
            package='fer_moveit',
            executable='fer_moveit_motion_server',
            name='fer_moveit_motion_server',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            # The robot model is for MoveIt Servo (/motion/follow).
            parameters=[params_file, *moveit_parameters(),
                        {'hardware': hardware, 'use_sim_time': hardware == 'mujoco'}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'hardware', default_value='mujoco', choices=list(HARDWARE),
            description="'real' (robot mode from franka_robot_state_broadcaster) or 'mujoco'."),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Node log level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description="Parameters; '' selects config/fer_moveit_motion_server.yaml."),
    ]
