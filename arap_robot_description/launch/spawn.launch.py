from os.path import join

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# CONSTANTS
PACKAGE_NAME = "arap_robot_description"
WAIT_PERIOD = 10.0


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
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    robot = LaunchConfiguration("robot").perform(context)
    use_ros2_control = (
        LaunchConfiguration("use_ros2_control").perform(context) == "true"
    )

    # Spawn robot at the given position
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # Robot State Publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "rsp.launch.py")]),
        launch_arguments={
            "sim_mode": "true",
            "robot_type": robot,
            "jsp_gui": "false",
            "use_rviz": "false",
        }.items(),
    )

    # Spawn the robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot,
            "-allow_renaming",
            "true",
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            str(roll),
            "-P",
            str(pitch),
            "-Y",
            str(yaw),
        ],
    )

    # ROS-Gazebo bridge
    bridge_params = join(pkg_share, "config", "gz_bridge.yaml")

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/came_one/image"],
        remappings=[("/came_one/image", "/came_one/color/image_raw")],
    )

    launch_nodes = [rsp, spawn_robot, ros_gz_bridge, ros_gz_image_bridge]

    if use_ros2_control:
        # Spawn the controller manager
        load_diff_drive_controller = RegisterEventHandler(
            event_handler=OnProcessExit(  # Wait for the robot to spawn
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=WAIT_PERIOD,
                        actions=[
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                output="screen",
                                arguments=[
                                    "diff_controller",
                                    "--controller-ros-args",
                                    "-r /diff_controller/cmd_vel:=/cmd_vel",
                                ],
                            )
                        ],
                    )
                ],
            )
        )

        load_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=WAIT_PERIOD,
                        actions=[
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                arguments=["joint_broad"],
                                output="screen",
                            )
                        ],
                    )
                ],
            )
        )

        launch_nodes.extend([load_diff_drive_controller, load_joint_state_broadcaster])

    return launch_nodes


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the robot visualization.

    This function sets up all necessary nodes and parameters for visualizing
    the robot in RViz, including:
    - Robot state publisher for broadcasting transforms
    - Joint state publisher for simulating joint movements
    - RViz for visualization

    Returns:
        LaunchDescription: Complete launch description for the visualization setup
    """

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="arap_robot",
            choices=["arap_robot"],
            description="Robot URDF file to load",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="X position of the robot",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="Y position of the robot",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.19",
            description="Z position of the robot",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0",
            description="Roll position of the robot",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0",
            description="Pitch position of the robot",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0",
            description="Yaw position of the robot",
        ),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="false",
            choices=["true", "false"],
            description="Use ros2_control if true",
        ),
        OpaqueFunction(function=launch_setup),
    ])
