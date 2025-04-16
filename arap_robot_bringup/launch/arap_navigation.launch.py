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
    AppendEnvironmentVariable,
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
    descr_pkg_share = get_package_share_directory(PACKAGE_NAME)
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
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    slam = LaunchConfiguration("slam").perform(context)
    debug = LaunchConfiguration("debug").perform(context)
    
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
    
    # # Add cmd_vel relay node if needed
    # cmd_vel_relay = Node(
    #     package=NAVIGATION_PACKAGE,
    #     executable="cmd_vel_relay",
    #     name="cmd_vel_relay",
    #     output="screen",
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )
    
    # Add map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'yaml_filename': map_yaml_file}
        ]
    )

    # Add lifecycle manager for map server
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'autostart': autostart.lower() == 'true'},
            {'node_names': ['map_server']}
        ]
    )

    params_file = join(nav_pkg_share, 'config', 'arap_nav2_default_params.yaml')

    # Add AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Add lifecycle manager for AMCL
    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'autostart': autostart.lower() == 'true'},
            {'node_names': ['amcl']}
        ]
    )

    # Navigation core components
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Navigation lifecycle manager
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'autostart': autostart.lower() == 'true'},
            {'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}
        ]
    )

    # SLAM Toolbox (delayed to ensure odometry is available)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'slam_params_file':params_file}.items()
    )

    delayed_slam = TimerAction(
        period=15.0,
        actions=[slam_launch]
    )
    
    # Launch the ROS 2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': join(nav_pkg_share, 'config', 'arap_nav2_default_params.yaml'),
            'autostart': autostart
        }.items()
    )

    # Map to Odom static transform [TEMPORARY SOLUTION]
    map_to_odom_tf = Node(
        condition=UnlessCondition(slam),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}]
    )

    # Debug node to print TF tree
    tf_debug = Node(
        condition=IfCondition(debug),  # Add a debug launch argument
        package='tf2_tools',
        executable='view_frames',
        name='view_frames',
        output='screen'
    )

    launch_nodes = [
        # map_to_odom_tf,
        set_model_path, 
        gazebo, 
        spawn, 
        ekf, 
        # delayed_slam,
        rviz,
        # cmd_vel_relay,
        map_server,
        map_lifecycle_manager,
        amcl,
        localization_lifecycle_manager,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        nav_lifecycle_manager,
        # nav2_launch,
        tf_debug,
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
        DeclareLaunchArgument(
            "slam",
            default_value="true",
            choices=["true", "false"],
            description="Whether to run SLAM"
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            choices=["true", "false"],
            description="Automatically start up the nav2 stack"
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