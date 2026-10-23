"""One command for the full simulation demo: Gazebo office + robot + RViz, then Nav2 on the saved map.

    ros2 launch omninav_sim navigation.launch.py
    ros2 launch omninav_sim navigation.launch.py nav_map:=/path/to/other_map.yaml

This REPLACES sim.launch.py - do not run both at the same time.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('omninav_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg, 'maps', 'office.yaml')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # Unique argument names, so nothing inside sim.launch.py or nav2_bringup can clash with them
    map_arg = DeclareLaunchArgument('nav_map', default_value=default_map,
                                    description='Map yaml saved from slam_toolbox')
    params_arg = DeclareLaunchArgument('nav_params', default_value=default_params,
                                       description='Nav2 parameters yaml')

    # Gazebo world, robot spawn, bridge, robot_state_publisher, lidar TF, RViz.
    # scoped=True keeps any launch arguments set inside sim.launch.py from leaking out here.
    sim = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')))
    ])

    # Nav2: map_server + AMCL + planner/controller/behaviours (no SLAM)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'map': LaunchConfiguration('nav_map'),
            'params_file': LaunchConfiguration('nav_params'),
            'autostart': 'true',
        }.items())

    return LaunchDescription([
        map_arg, params_arg,
        LogInfo(msg=['Nav2 map:    ', LaunchConfiguration('nav_map')]),
        LogInfo(msg=['Nav2 params: ', LaunchConfiguration('nav_params')]),
        sim,
        # Nav2 needs /clock, /tf and /scan from the sim, so give the sim a head start
        TimerAction(period=8.0, actions=[nav2]),
    ])