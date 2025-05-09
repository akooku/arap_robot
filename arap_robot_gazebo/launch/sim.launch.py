"""
File: sim.launch.py
Project: Limo Description
File Created: Monday, 31st March 2025 10:42:15 AM
Author: nknab
Email: adverb_rushed_09@icloud.com
Version: 1.0
Brief: Launch file to start the simulation and spawn the Limo robot.
-----
Last Modified: Monday, 31st March 2025 8:00:07 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# CONSTANTS
DESCR = "arap_robot_description"
GAZEBO = "arap_robot_gazebo"


def launch_setup(context: LaunchContext) -> list:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context object to get the launch configuration

    Returns
    -------
    list
        The list of launch nodes to execute

    """

    # Get the package share directory
    pkg_descr = FindPackageShare(package=DESCR).find(DESCR)
    pkg_gz = FindPackageShare(package=GAZEBO).find(GAZEBO)

    # Get the launch configuration variables
    world = LaunchConfiguration("world").perform(context)

    # Append GZ_SIM_RESOURCE_PATH - MOVE THIS BEFORE CREATING THE LAUNCH ACTIONS
    model_path = join(pkg_gz, "models")
    set_model_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    # Gazebo launch file
    world_filepath = join(pkg_gz, "worlds", f"{world}.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ]),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_filepath],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_descr, "launch", "spawn.launch.py")]),
    )

    # Configure the bridge with QoS settings
    bridge_config = join(pkg_gz, "config", "gazebo_bridge_params.yaml")
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[bridge_config],
        output='screen'
    )

    return [set_model_path, gazebo, spawn, bridge]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to spawn a limo robot

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="cafe",
            choices=["empty", "cafe", "house", "ashesi"],
            description="The world to load",
        ),
        OpaqueFunction(function=launch_setup),
    ])