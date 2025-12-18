from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Basic Config
    package_name = 'HumanoidCtrl'
    HumanoidCtrl_dir = get_package_share_directory(package_name)
    basic_config = os.path.join(HumanoidCtrl_dir, 'config', 'BasicParamter.yaml')

    try:
        with open(basic_config, 'r') as f:
            basic_config_params = yaml.safe_load(f)
            params = basic_config_params[package_name]['ros__parameters']
            robot_type = params.get('RobotType', 'Ti5Robot')
            joint_topic = params.get('OutputTopicName', '/joint_states')
    except Exception as e:
        print(f"Error loading basic config: {e}")
        robot_type = 'Ti5Robot'
        joint_topic = '/joint_states'
    
    robot_config = os.path.join(HumanoidCtrl_dir, 'config', robot_type + '.yaml')

    # RViz Part
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
        remappings=[('/joint_states', joint_topic)]
    )

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

    # Controller Part
    controller_node = Node(
            package=package_name,
            executable=package_name,
            name=package_name,
            output='screen',
            parameters=[basic_config,
                        robot_config]
        )

    return LaunchDescription([
        # RViz Part
        robot_state_publisher_node,
        rviz_node,

        # Controller Part
        controller_node
    ])