#!/usr/bin/env python3
"""
File: sim.launch.py
Project: ARAP Robot Simulation
File Created: Monday, 31st March 2025 10:42:15 AM
Author: Ako (a.k.a. nknab)
Email: adverb_rushed_09@icloud.com
Version: 1.0
Brief: Launch file to start the simulation, EKF localization, navigation, and spawn the robot.
-----
Last Modified: Monday, 31st March 2025 8:00:07 PM
Modified By: Ako
-----
Copyright ©2025 Ako
"""
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

# === Constants === #
PACKAGE_NAME = "arap_robot_description"
LOCALIZATION_PACKAGE = "arap_robot_localization"
GAZEBO_PACKAGE = "arap_robot_gazebo"
NAVIGATION_PACKAGE = "arap_robot_navigation"

def launch_setup(context: LaunchContext) -> list:
    # descr_pkg_share = get_package_share_directory(PACKAGE_NAME)
    loc_pkg_share = get_package_share_directory(LOCALIZATION_PACKAGE)
    gz_pkg_share = get_package_share_directory(GAZEBO_PACKAGE)
    nav_pkg_share = get_package_share_directory(NAVIGATION_PACKAGE)
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Get Nav2 package dir for the launch files
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = join(nav2_dir, 'launch')
    
    # Launch configurations
    world = LaunchConfiguration("world").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    map_yaml_file = LaunchConfiguration("map").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    use_composition = LaunchConfiguration("use_composition").perform(context)
    use_respawn = LaunchConfiguration("use_respawn").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    slam = LaunchConfiguration("slam").perform(context)

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
    
    # Launch Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(gz_pkg_share, "launch", "sim.launch.py")
        ]),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time
        }.items(),
    )

    # RViz
    rviz_config_file = join(nav_pkg_share, "rviz", f"{rviz_config}.rviz")

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )
    
    # Launch the ROS 2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(nav_pkg_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': join(nav_pkg_share, 'config', 'arap_nav2_default_params.yaml'),
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'lifecycle_nodes': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor', 'smoother_server']
        }.items()
    )

    launch_nodes = [
        ekf,
        gz_launch, 
        rviz_cmd, 
        nav2_launch,
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
            default_value=join(get_package_share_directory(NAVIGATION_PACKAGE), "maps", "my_cafe_map_edited.yaml"),
            description="Full path to map file to load"
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run SLAM"
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            choices=["true", "false"],
            description="Automatically start up the nav2 stack"
        ),

        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
            choices=["True", "False"],
            description="Use composition for nav2 nodes"
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            choices=["True", "False"],
            description="Use respawn for nav2 nodes when composition is disabled"
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
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            choices=["true", "false"],
            description="Enable debugging output"
        ),
        
        OpaqueFunction(function=launch_setup)
    ])