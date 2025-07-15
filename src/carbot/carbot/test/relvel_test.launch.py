from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carbot',
            executable='fake_position_publisher',
            name='fake_position_publisher',
            output='screen'
        ),
        Node(
            package='carbot',
            executable='relvel_and_velstop',
            name='relvel_and_velstop',
            output='screen',
            parameters=[
                {'velocity_threshold': 0.05},
                {'stop_time_threshold': 1.0}
            ]
        )
    ])
