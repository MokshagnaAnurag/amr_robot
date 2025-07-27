import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'articubot_one'

    # Robot Description
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare(package_name),
            'description',
            'robot.urdf.xacro'
        ]),
        ' sim_mode:=false'
    ])
    robot_description = {'robot_description': robot_description_content}

    # 1. Robot State Publisher + Joint State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # 2. twist_mux
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # 3. Controller Manager (ROS2 Control)
    controller_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="screen"
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # 4. RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # 5. Motor Controller
    motor_controller_node = Node(
        package='articubot_gpio_driver',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    # 6. SLAM Toolbox Launch (Optional, if needed)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'online_async_launch.py'
            )
        ])
    )

    # 7. Navigation2 Launch (Optional, if needed)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'navigation_launch.py'
            )
        ])
    )

    # 8️⃣ Static Transform Publisher: map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 9️⃣ Static Transform Publisher: base_link -> laser
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        #rplidar_node,
        motor_controller_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        static_tf_map_odom,
        static_tf_base_laser,
    ])

