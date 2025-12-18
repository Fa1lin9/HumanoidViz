#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_type = 'Ti5Robot'
    joint_topic_arg = DeclareLaunchArgument(
        'jointTopic',
        default_value = '/joint_states'
    )

    description_package = robot_type + "Description"
    description_urdf = robot_type + ".urdf"
    description_xacro = robot_type + ".urdf.xacro"
    description_rviz = robot_type + ".rviz"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                # [FindPackageShare(description_package), "config", description_xacro]
                [FindPackageShare(description_package), "urdf", description_urdf]
            )
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type = str)}

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('/joint_states', LaunchConfiguration('jointTopic'))]
    )
	
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher'
    )

    
    # RViz 节点
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        'rviz',
        description_rviz
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_topic_arg,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node
    ])

