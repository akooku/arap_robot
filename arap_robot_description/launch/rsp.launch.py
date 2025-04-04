from os.path import join

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# CONSTANTS
PACKAGE_NAME = "arap_robot_description"


def launch_setup(context: LaunchContext) -> list[Node]:
    """
    Function to setup the robot_state_publisher node

    Parameters
    ----------
    context : LaunchContext
        The launch context

    Returns
    -------
    list[Node]
        The robot_state_publisher node

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    use_ros2_control = LaunchConfiguration("use_ros2_control").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    jsp_gui = LaunchConfiguration("jsp_gui").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    drive_system = LaunchConfiguration("drive_system").perform(context)

    # Process the xacro file to get the URDF
    xacro_file = join(pkg_share, "urdf", "control", f"{drive_system}", "system.urdf.xacro")

    robot_description = Command([
        f"xacro {xacro_file} ",
        f"use_ros2_control:={use_ros2_control} ",
        f"sim_mode:={sim_mode} ",
        f"robot_name:={robot_type} ",
        f"prefix:={prefix} ",
        f"drive_system:={drive_system}",
    ])

    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "sim_mode": sim_mode.lower() == "true",
                "use_sim_time": sim_mode.lower() == "true",
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(jsp_gui),
    )

    # RViz
    rviz_config_file = join(pkg_share, "rviz", f"{rviz_config}.rviz")

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return [
        robot_state_publisher,
        joint_state_publisher,
        rviz_cmd,
    ]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to start the robot_state_publisher node

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false",
            choices=["true", "false"],
            description="Determine if the robot is in simulation mode",
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="arap_robot",
            choices=["arap_robot"],
            description="Robot URDF file to load",
        ),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="false",
            choices=["true", "false"],
            description="Use ROS2 control",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix to add to the robot name",
        ),
        DeclareLaunchArgument(
            "jsp_gui",
            default_value="true",
            choices=["true", "false"],
            description="Launch the joint state publisher GUI",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="arap_robot_description",
            choices=["arap_robot_description", "nav2_default_view"],
            description="Name of the RViz configuration file",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Whether to start RVIZ",
        ),
        DeclareLaunchArgument(
            "drive_system",
            default_value="diff",
            choices=["diff", "mecanum", "ackermann"],
            description="Drive system to use",
        ),
        OpaqueFunction(function=launch_setup),
    ])
