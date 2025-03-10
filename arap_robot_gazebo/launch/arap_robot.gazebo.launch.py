#!/usr/bin/env python3
"""
Launch Gazebo simulation with a 4-wheeled differential drive robot.

This launch file sets up a complete ROS 2 simulation environment with Gazebo
for the arap robot featuring four wheels and an RGBD camera (cam_1).

:author: Ako Eyo Oku
:date: March 9, 2025
"""

import os
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the Gazebo simulation.

    This function sets up all necessary parameters, paths, and nodes required to launch
    the Gazebo simulation with the arap robot.

    Returns:
        LaunchDescription: A complete launch description for the simulation.
    """
    # Package names
    package_name_gazebo = 'arap_robot_gazebo'
    package_name_description = 'arap_robot_description'
    package_name_bringup = 'arap_robot_bringup'

    default_robot_name = 'arap_robot'
    default_world_file = 'empty.world'
    gazebo_models_path = 'models'
    gazebo_worlds_path = 'worlds'
    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'
    rviz_config_filename = 'arap_robot_gazebo_sim.rviz'

    # Get package paths
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
    pkg_share_bringup = FindPackageShare(package=package_name_bringup).find(package_name_bringup)

    default_ros_gz_bridge_config_file_path = os.path.join(pkg_share_gazebo, ros_gz_bridge_config_file_path)
    default_rviz_config_path = PathJoinSubstitution([pkg_share_gazebo, 'rviz', rviz_config_filename])
    gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)

    # Launch configuration variables
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        pkg_share_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    # Robot spawn pose arguments
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument('robot_name', default_value=default_robot_name, description='Robot name')
    declare_world_cmd = DeclareLaunchArgument('world_file', default_value=default_world_file, description='Gazebo world file')
    declare_use_gazebo_cmd = DeclareLaunchArgument('use_gazebo', default_value='true', description='Enable Gazebo simulation')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='true', description='Enable RViz visualization')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated time')
    declare_enable_odom_tf_cmd = DeclareLaunchArgument('enable_odom_tf', default_value='true', description='Enable odometry transform')
    declare_load_controllers_cmd = DeclareLaunchArgument('load_controllers', default_value='true', description='Load robot controllers')

    # Robot State Publisher (if enabled)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={
            'enable_odom_tf': enable_odom_tf,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )

    # Load controllers (if enabled)
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share_bringup, 'launch', 'load_ros2_controllers.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(load_controllers)
    )

    # Set Gazebo model path
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_models_path)

    # Start Gazebo simulation
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -s -v 4 ', world_path])])

    # Start Gazebo client (GUI) if not headless
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g ']}.items(),
        condition=IfCondition(PythonExpression(['not ', headless])))

    # Start ROS-Gazebo Bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': default_ros_gz_bridge_config_file_path}],
        output='screen'
    )

    # Start camera bridge (for cam_1)
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/cam_1/image'],
        remappings=[('/cam_1/image', '/cam_1/color/image_raw')]
    )

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_load_controllers_cmd)

    # Add pose arguments
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0', description='Initial x position'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0', description='Initial y position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.05', description='Initial z position'))
    ld.add_action(DeclareLaunchArgument('roll', default_value='0.0', description='Initial roll'))
    ld.add_action(DeclareLaunchArgument('pitch', default_value='0.0', description='Initial pitch'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw'))

    # Add actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld