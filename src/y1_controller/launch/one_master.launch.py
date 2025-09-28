import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory('y1_controller'), 'config', 'one_master.yaml')

arm_node = Node(
    package='y1_controller',
    executable='y1_controller',
    name='right_master_arm',
    output='screen',
    parameters=[params_file]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        arm_node,
    ])