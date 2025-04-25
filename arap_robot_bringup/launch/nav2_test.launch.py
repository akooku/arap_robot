#!/usr/bin/env python3
"""
Launch file for directly using Nav2 with AMCL localization and visualization.
This launch file uses the standard Nav2 bringup to ensure particle cloud visualization works.
"""

import os
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# === Constants === #
PACKAGE_NAME = "arap_robot_description"
LOCALIZATION_PACKAGE = "arap_robot_localization"
GAZEBO_PACKAGE = "arap_robot_gazebo"
NAVIGATION_PACKAGE = "arap_robot_navigation"

def launch_setup(context: LaunchContext) -> list:
    descr_pkg_share = get_package_share_directory(PACKAGE_NAME)
    loc_pkg_share = get_package_share_directory(LOCALIZATION_PACKAGE)
    gz_pkg_share = get_package_share_directory(GAZEBO_PACKAGE)
    nav_pkg_share = get_package_share_directory(NAVIGATION_PACKAGE)
    
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav_launch_dir = join(nav2_bringup_dir, 'launch')
    
    # Launch configurations
    world = LaunchConfiguration("world").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    map_yaml_file = LaunchConfiguration("map").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    
    # World path
    world_path = join(gz_pkg_share, "worlds", f"{world}.world")
    
    # Append GZ_SIM_RESOURCE_PATH
    model_path = join(gz_pkg_share, "models")
    set_model_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
    
    # Launch Gazebo with specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ]),
        launch_arguments={
            "gz_args": f"-r -v4 {world_path}",
            "on_exit_shutdown": "true"
        }.items(),
    )
    
    # Spawn the robot immediately
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(descr_pkg_share, "launch", "spawn.launch.py")
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    # Start EKF immediately
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(loc_pkg_share, "launch", "ekf.launch.py")
        ]),
        launch_arguments={
            "ekf_config_file": join(loc_pkg_share, "config", "ekf.yaml"),
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    # RViz
    rviz_config_file = join(nav_pkg_share, "rviz", f"{rviz_config}.rviz")

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )
    
    # Launch Nav2 navigation stack with localization
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(nav_launch_dir, 'bringup_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': join(nav_pkg_share, 'config', 'arap_nav2_default_params.yaml'),
            'autostart': 'true',
        }.items()
    )
    
    # Delay Nav2 startup to ensure Gazebo and robot are fully loaded
    delayed_nav2 = TimerAction(
        period=50.0,  # 10 second delay
        actions=[nav2_bringup_cmd]
    )

    launch_nodes = [
        set_model_path, 
        gazebo, 
        spawn, 
        ekf, 
        rviz,
        delayed_nav2
    ]

    return launch_nodes

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # World configuration
        DeclareLaunchArgument(
            "world",
            default_value="empty",
            choices=["empty", "house", "cafe", "ashesi"],
            description="World to load (e.g., empty, house, cafe)"
        ),
        
        # Simulation configuration
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use simulation clock"
        ),
        
        # Navigation configuration
        DeclareLaunchArgument(
            "map",
            default_value=join(get_package_share_directory(NAVIGATION_PACKAGE), "maps", "cafe_map.yaml"),
            description="Full path to map file to load"
        ),
        
        # Visualization
        DeclareLaunchArgument(
            "rviz_config",
            default_value="nav2_default_view",
            choices=["nav2_default_view", "arap_robot_description"],
            description="Name of the RViz configuration file",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Whether to start RViz"
        ),
        
        OpaqueFunction(function=launch_setup)
    ])