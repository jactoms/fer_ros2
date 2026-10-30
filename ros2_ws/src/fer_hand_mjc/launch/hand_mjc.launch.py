"""Standalone MuJoCo hand: own simulation, hand position on /hand_mjc/position, and RViz."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> List[Node]:

    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context).lower() == 'true'
    motion = LaunchConfiguration('motion').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    overrides = {'viewer': viewer}
    if motion:
        overrides['motion'] = motion

    nodes = [
        Node(
            package='fer_hand_mjc',
            executable='hand_mjc_node',
            name='fer_hand_mjc',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[params_file, overrides],
        )
    ]
    if rviz:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level],
            ))
    return nodes


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'config', 'hand_mjc.yaml']),
            description='Parameter file of the MuJoCo hand.'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='true: open the MuJoCo viewer; false: headless.'),
        DeclareLaunchArgument(
            'motion', default_value='',
            description='Overrides the motion in params_file: static, sine, random.'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='true: start RViz.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'rviz', 'hand_mjc.rviz']),
            description='RViz configuration file.'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])
