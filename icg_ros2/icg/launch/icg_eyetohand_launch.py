from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    icg_params = os.path.join(
        get_package_share_directory('icg_ros'),
        'config',
        'icg_eyetohand.yaml'
    )

    icg_node = Node(
        package='icg_ros',
        executable='icg_eyetohand',
        name='icg_eyetohand',
        output='screen',
        parameters=[icg_params]
    )
    return LaunchDescription([
            icg_node
        ])