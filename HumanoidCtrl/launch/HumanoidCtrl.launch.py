from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Type Definition
    robot_type = 'Ti5Robot'

    # Directory
    HumanoidCtrl_dir = get_package_share_directory('HumanoidCtrl')

    # Configuration
    basic_config = os.path.join(HumanoidCtrl_dir, 'config', 'BasicParamter.yaml')
    robot_config = os.path.join(HumanoidCtrl_dir, 'config', robot_type + '.yaml')

    return LaunchDescription([
        Node(
            package='HumanoidCtrl',
            executable='HumanoidCtrl',
            name='HumanoidCtrl',
            output='screen',
            parameters=[basic_config,
                        robot_config]
        )
    ])
