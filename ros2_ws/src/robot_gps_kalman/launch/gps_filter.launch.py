from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_gps_kalman',
            executable='gps_filter',
            name='robot_gps_kalman',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
