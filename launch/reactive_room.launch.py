import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    package_share = get_package_share_directory(
        'reactive_turtlebot'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    # Our custom Gazebo world.
    world_file = os.path.join(
        package_share,
        'worlds',
        'reactive_room.sdf'
    )

    # Start Gazebo directly with our world file.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': world_file + ' -r -v 4'
        }.items()
    )

    # Bridge Gazebo simulation time to ROS 2 /clock.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/world/reactive_room/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/reactive_room/clock', '/clock')
        ],
        output='screen'
    )

    # Bridge the namespaced TurtleBot 4 RPLIDAR to ROS 2.
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='reactive_lidar_bridge',
        arguments=[
            '/world/reactive_room/model/robot1/turtlebot4/'
            'link/rplidar_link/sensor/rplidar/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            (
                '/world/reactive_room/model/robot1/turtlebot4/'
                'link/rplidar_link/sensor/rplidar/scan',
                '/robot1/scan'
            )
        ],
        output='screen'
    )

    # Spawn the TurtleBot 4 into the already-running world.
    turtlebot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_spawn.launch.py'
            )
        ),
        launch_arguments={
            'model': 'standard',
            'use_sim_time': 'true',
            'rviz': 'false',
            'localization': 'false',
            'slam': 'false',
            'nav2': 'false',
            'x': '1.5',
            'y': '3.5',
            'z': '0.1',
            'yaw': '0.0',
            'namespace': 'robot1',
        }.items()
    )


    reactive_node = Node(
        package='reactive_turtlebot',
        executable='reactive_turtlebot',
        name='reactive_turtlebot',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/robot1/diffdrive_controller/cmd_vel'),
            ('/scan', '/robot1/scan'),
            ('/odom', '/robot1/odom'),
            ('/bumper_contact', '/robot1/bumper_contact'),
        ],
    )

    mapper_node = Node(
        package='reactive_turtlebot',
        executable='occupancy_grid_mapper',
        name='occupancy_grid_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/scan', '/robot1/scan'),
            ('/odom', '/robot1/odom'),
        ],
    )

     # Delay the start of the reactive_node and mapper_node to ensure that the TurtleBot has spawned and the bridges are established.
     # Removing teh timer action to see if it helps with the issue of the turtlebot not moving in the simulation.
    return LaunchDescription([
        gazebo,
        clock_bridge,
        turtlebot_spawn,
        lidar_bridge,
        reactive_node,
        mapper_node
    ])
