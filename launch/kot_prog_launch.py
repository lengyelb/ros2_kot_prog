from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    factory_node = Node(
            package='ros2_kot_prog',
            executable='marker_factory',
            name='marker_factory_node',
        )

    psm_grasp_node = Node(
            package='ros2_kot_prog',
            executable='psm_grasp',
            name='psm_grasp_node',
        )

    return LaunchDescription([
        factory_node,
        psm_grasp_node
    ])
