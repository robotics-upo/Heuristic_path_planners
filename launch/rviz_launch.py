from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            #arguments=['-d' + os.path.join(get_package_share_directory('package_name'), 'config', 'planners.rviz')]
        )
    ])
    
