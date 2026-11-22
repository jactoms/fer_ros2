"""
Check of the Cartesian streamer (WP2 step 2): moves the TCP a short distance along one axis.

Needs the MuJoCo stack with effort_trajectory_controller active; the motion server must not
run at the same time (both would command the arm).
"""
import os
import sys
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('fer_moveit'), 'launch'))
from moveit_parameters import moveit_parameters, servo_parameters  # noqa: E402


def launch_setup(context, *args, **kwargs) -> List[Node]:
    params_file = os.path.join(
        get_package_share_directory('fer_moveit'), 'config', 'fer_moveit_motion_server.yaml')
    check = {name: LaunchConfiguration(name).perform(context)
             for name in ('axis', 'distance', 'speed')}
    return [Node(
        package='fer_moveit',
        executable='fer_moveit_streamer_check',
        name='fer_moveit_streamer_check',
        output='both',
        # Quoted on the command line: in the parameter file launch_ros writes, an unquoted y
        # would be read as the YAML boolean true.
        ros_arguments=['-p', f"axis:='{check['axis']}'"],
        parameters=moveit_parameters() + [
            servo_parameters(params_file),
            {'distance': float(check['distance']),
             'speed': float(check['speed']), 'use_sim_time': True}],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('axis', default_value='x', choices=['x', 'y', 'z'],
                              description='Axis of the base frame to move along.'),
        DeclareLaunchArgument('distance', default_value='0.05',
                              description='m, signed; the arm moves this far and stops.'),
        DeclareLaunchArgument('speed', default_value='0.05', description='m/s, > 0.'),
        OpaqueFunction(function=launch_setup),
    ])
