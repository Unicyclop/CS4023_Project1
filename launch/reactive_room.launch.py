import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('reactive_turtlebot')

    turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    # TurtleBot4's sim.launch.py automatically adds ".sdf".
    # Therefore, give it the absolute world path WITHOUT the extension.
    world = os.path.join(
        package_share,
        'worlds',
        'reactive_room'
    )

    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_gz.launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'x': '0.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0',
        }.items()
    )

    # Bridge the TurtleBot4 RPLIDAR from Gazebo to ROS 2 /scan.
    lidar_gz_topic = (
        '/world/reactive_room'
        '/model/turtlebot4'
        '/link/rplidar_link'
        '/sensor/rplidar/scan'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            lidar_gz_topic +
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            (lidar_gz_topic, '/scan')
        ]
    )

    return LaunchDescription([
        turtlebot_launch,
        lidar_bridge
    ])
