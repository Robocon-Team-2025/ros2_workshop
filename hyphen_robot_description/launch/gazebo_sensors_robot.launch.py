import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

ROBOT_NAME = "hyphen_robot"  # Modify here to specify your own Robot Name
PACKAGE_NAME = "hyphen_robot_description"  # This should match your package name
URDF_FILE_PATH = "urdf/hyphen_robot_sensors.urdf"  # This part specifies the location of the urdf for your robot model


def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    default_model_path = os.path.join(pkg_share, URDF_FILE_PATH)

    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    world_file_name = 'rbc2025.world'  # Replace with your custom world file
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # ROBOT STATE PUBLISHER is a node that serves a critical function in robotic simulation and visualization by publishing the state (positions, velocities, and efforts) of all joints in a robot model.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    # JOINT STATE PUBLISHER
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[default_model_path],
    )

    # Robot Spawner
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", ROBOT_NAME, "-topic", "robot_description"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription(
        [
            # DECLARE LAUNCH ARGUMENTS
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="world",
                default_value=world_path,
                description="Absolute path to gazebo world file",
            ),
            # Gazebo
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    LaunchConfiguration('world'),
                ],
                output="screen",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            spawn_entity,
            # rviz_node,
        ]
    )
