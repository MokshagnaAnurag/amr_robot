from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = '/home/moksh/ros2_ws/src/my_robot_description/urdf/robot.urdf'

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Lidar Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 115200}
            ],
            output='screen'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False
            }],
            output='screen'
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/moksh/ros2_ws/src/my_slam_toolbox/config/slam_config.rviz'],
            output='screen'
        )
    ])

