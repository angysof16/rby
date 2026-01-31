import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


ARGUMENTS = [
    DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Enable joint_state_publisher_gui",
    ),
    DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("rby_description"),
                "urdf",
                "robots",
                "rby_standalone.urdf.xacro",
            ]
        ),
        description="Path to the robot URDF/Xacro",
    ),
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    ),
    DeclareLaunchArgument(
        name="rviz_config",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("rby_description"),
                "launch",
                "config.rviz",
            ]
        ),
        description="Path to RViz config file",
    ),
]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    
    # Get launch configurations
    model = LaunchConfiguration("model")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    
    # Robot State Publisher - publishes TF from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", model]),
            }
        ],
    )
    
    # Joint State Publisher - publishes joint states
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(gui),
    )
    
    # Joint State Publisher GUI - allows manual control of joints
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(gui),
    )
    
    # RViz - visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Add all nodes to launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    
    return ld