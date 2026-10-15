from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    reactive_node = Node(
        package='reactive_turtlebot',
        executable='reactive_turtlebot',
        name='reactive_turtlebot',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
    )

    mapper_node = Node(
        package='reactive_turtlebot',
        executable='occupancy_grid_mapper',
        name='occupancy_grid_mapper',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
    )

    return LaunchDescription([
        reactive_node,
        mapper_node,
    ])
