import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('omninav_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'office.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'omninav.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'omninav.rviz')


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s --headless-rendering -v 1 {world_file}'}.items()
    )

    
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

   

    robot_state_publisher = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]

    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'omninav',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen'
    )

    bridge= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'}
    )

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.1', '--y', '0', '--z', '0.15',
                    '--frame-id', 'base_footprint', '--child-frame-id', 'omninav/base_footprint/lidar'],
        parameters=[{'use_sim_time': True}]

    )

    wheel_anim = Node(
        package='omninav_sim',
        executable='wheel_animator.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim, robot_state_publisher, spawn_robot, bridge, rviz, static_tf_lidar, wheel_anim
    ])