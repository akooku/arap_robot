#!/usr/bin/env python3
"""
Simple test launch file to verify SLAM functionality
"""
from os.path import join
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    AppendEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories
    pkg_gazebo = get_package_share_directory('arap_robot_gazebo')
    pkg_description = get_package_share_directory('arap_robot_description')
    pkg_navigation = get_package_share_directory('arap_robot_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Append GZ_SIM_RESOURCE_PATH
    model_path = join(pkg_gazebo, "models")
    set_model_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
    
    # 1. Start Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r -v4 ' + os.path.join(pkg_gazebo, 'worlds', 'cafe.world'),
            'on_exit_shutdown': 'true'
        }.items(),
    )
    
    # 2. Spawn robot with differential drive
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_description, 'launch', 'spawn.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x': '0.0',
            'y': '0.0',
            'z': '0.2'
        }.items(),
    )

    # 2. SLAM Toolbox configuration
    slam_params = {
        'use_sim_time': True,
        'map_frame': 'map',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'map_update_interval': 1.0,
        'resolution': 0.05,
        'max_laser_range': 20.0
    }

    slam_params_file = os.path.join(pkg_navigation, 'config', 'arap_nav2_default_params.yaml')

    # SLAM Toolbox (delayed to ensure odometry is available)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'slam_params_file':slam_params_file}.items()
    )

    delayed_slam = TimerAction(
        period=15.0,
        actions=[slam_launch]
    )

    # 4. RViz for visualization
    rviz_config = os.path.join(pkg_navigation, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 5. Add a teleop node to easily move the robot
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',
        output='screen'
    )
    
    # 6. Debug information
    tf_debug = Node(
        package='tf2_ros', 
        executable='tf2_echo',
        name='tf_debug',
        arguments=['odom', 'base_link'],
        output='screen'
    )
    
    return LaunchDescription([
        set_model_path,
        gazebo,
        delayed_slam,
        spawn_robot,
        rviz,
        teleop,
        tf_debug
    ])