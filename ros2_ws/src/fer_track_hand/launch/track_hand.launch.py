"""Hand tracker controller; with hand:=true also the fer_hand_mjc simulation (and its RViz)."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> list:

    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    hand = LaunchConfiguration('hand').perform(context).lower() == 'true'

    actions = []
    if hand:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'launch', 'hand_mjc.launch.py'])),
            launch_arguments={
                'log_level': log_level,
                'params_file': LaunchConfiguration('hand_params_file').perform(context),
                'viewer': LaunchConfiguration('viewer').perform(context),
                'motion': LaunchConfiguration('motion').perform(context),
                'rviz': LaunchConfiguration('rviz').perform(context),
                'rviz_config': LaunchConfiguration('rviz_config').perform(context),
            }.items(),
        ))
    actions.append(Node(
        package='fer_track_hand',
        executable='track_hand_node',
        name='fer_track_hand',
        output='both',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
    ))
    return actions


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_track_hand'), 'config', 'track_hand.yaml']),
            description='Parameter file of the tracker.'),
        DeclareLaunchArgument(
            'hand', default_value='false',
            description='false: track in a fer_hand_mjc simulation that is already running; '
                        'true: also start the simulation here.'),
        # Forwarded to fer_hand_mjc's hand_mjc.launch.py with hand:=true.
        DeclareLaunchArgument(
            'hand_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'config', 'hand_mjc.yaml']),
            description='With hand:=true: parameter file of the simulation.'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='With hand:=true: true opens the MuJoCo viewer; false: headless.'),
        DeclareLaunchArgument(
            'motion', default_value='',
            description='With hand:=true: hand motion, overrides hand_params_file: '
                        'static, sine, random.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='With hand:=true: true starts RViz.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'rviz', 'hand_mjc.rviz']),
            description='With hand:=true: RViz configuration file.'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])
