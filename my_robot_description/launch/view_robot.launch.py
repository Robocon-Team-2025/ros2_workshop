import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackage
import os

ROBOT_NAME = "hyphen"
PACKAGE_NAME = "my_robot_description"
URDF_FILE_PATH = 'urdf/my_simple_robot.urdf'

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    default_model_path = os.path.join(pkg_share, URDF_FILE_PATH)

    # TODO ADD Rviz Config Path
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz') 

    # ROBOT STATE PUBLISHER is a node that serves a critical function in robotic simulation and visualization by publishing the state (positions, velocities, and efforts) of all joints in a robot model. 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    # JOINT STATE PUBLISHER
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # JOINT STATE PUBLISHER GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    # TODO ADD RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        # DECLARE LAUNCH ARGUMENTS
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        # TODO ADD RVIZ ARGUMENT
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        
        # TODO Add RVIZ node call
        rviz_node,
    ])