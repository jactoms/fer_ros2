"""
MuJoCo hand: own simulation, hand position on /hand_mjc/position and TF world -> hand.

parent_frame (default base) links the hand's world frame to the robot with a static identity
transform parent_frame -> world, so the robot side (/motion/follow, the robot RViz) sees the
hand. world must be the frame_id in params_file.
"""
import math
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs) -> List[Node]:

    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context).lower() == 'true'
    motion = LaunchConfiguration('motion').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    parent_frame = LaunchConfiguration('parent_frame').perform(context)
    hand_speed = LaunchConfiguration('hand_speed').perform(context)

    overrides = {'viewer': viewer}
    if motion:
        overrides['motion'] = motion
    if hand_speed:
        # random: speed in m/s directly. sine: peak speed 2*pi*f*|amplitude| -> solve for f,
        # with the amplitude from params_file.
        speed = float(hand_speed)
        if speed <= 0.0:
            raise ValueError('hand_speed must be > 0')
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)['fer_hand_mjc']['ros__parameters']
        amplitude = math.sqrt(sum(a * a for a in params.get('sine_amplitude', [0.1, 0.25, 0.04])))
        overrides['random_speed'] = speed
        overrides['sine_frequency'] = speed / (2.0 * math.pi * amplitude)

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
    if parent_frame:
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='hand_mjc_world_link',
                output='log',
                arguments=['--frame-id', parent_frame, '--child-frame-id', 'world',
                           '--ros-args', '--log-level', log_level],
            ))
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
            'hand_speed', default_value='',
            description="Hand speed in m/s; '' keeps params_file. random: random_speed; "
                        'sine: peak speed (sets sine_frequency from sine_amplitude).'),
        DeclareLaunchArgument(
            'parent_frame', default_value='base',
            description="Static identity transform <parent_frame> -> world; '' for none "
                        '(hand alone, without the robot).'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true: start the hand RViz (fixed frame world). With the robot, its '
                        'RViz shows the hand already.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_hand_mjc'), 'rviz', 'hand_mjc.rviz']),
            description='RViz configuration file.'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])
