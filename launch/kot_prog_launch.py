from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    v = DeclareLaunchArgument(
        'v',
        default_value='0.005',
        description='The speed used by the robot during TCP movements'
    )

    dt = DeclareLaunchArgument(
        'dt',
        default_value='0.01',
        description='dt or loop rate'
    )

    omega = DeclareLaunchArgument(
        'omega',
        default_value='0.1',
        description='The angular speed to use durin JAW movements'
    )

    tcp_offset = DeclareLaunchArgument(
        'tcp_offset',
        default_value='0.008',
        description='The offset between the tool center point and where the jaw grabs'
    )

    grab_error = DeclareLaunchArgument(
        'grab_error',
        default_value='0.002',
        description='The maximum allowed error between tcp and different object to consider them grabbed'
    )

    shape = DeclareLaunchArgument(
        'shape',
        default_value='X',
        description='The desired shape to be drawn using the markers (X , +, -)'
    )

    debounce_timeout = DeclareLaunchArgument(
        'debounce_timeout',
        default_value='2',
        description='The debounce timeout between two marker requests'
    )

    v_value = LaunchConfiguration('v')
    dt_value = LaunchConfiguration('dt')
    omega_value = LaunchConfiguration('omega')
    tcp_offset_value = LaunchConfiguration('tcp_offset')
    grab_error_value = LaunchConfiguration('grab_error')
    shape_value = LaunchConfiguration('shape')
    debounce_timeout_value = LaunchConfiguration('debounce_timeout')

    factory_node = Node(
            package='ros2_kot_prog',
            executable='marker_factory',
            name='marker_factory_node',
            parameters=[{
                'grab_error': grab_error_value,
                'tcp_offset': tcp_offset_value,
                'debounce_timeout': debounce_timeout_value,
            }]
        )

    psm_grasp_node = Node(
            package='ros2_kot_prog',
            executable='psm_grasp',
            name='psm_grasp_node',
            parameters=[{
                'v': v_value,
                'dt': dt_value,
                'omega': omega_value,
                'tcp_offset': tcp_offset_value,
                'shape': shape_value,
            }]
        )

    return LaunchDescription([
        v,
        dt,
        omega,
        tcp_offset,
        grab_error,
        shape,
        debounce_timeout,
        factory_node,
        psm_grasp_node
    ])
