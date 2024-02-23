from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wit_ros_imu',
            executable='wit_imu_ctrl.py',
            name='imu',
            output='screen'
        )
    ])