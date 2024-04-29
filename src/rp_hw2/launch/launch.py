from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_hw2',
            executable='node1',
            name='node1',
            output='screen',
            parameters=[{'university_id': 2019741067}]
        ),
        Node(
            package='rp_hw2',
            executable='node2',
            name='node2',
            output='screen',  
            parameters=[{'multiplier_values': [4, 5]}]
        )
    ])