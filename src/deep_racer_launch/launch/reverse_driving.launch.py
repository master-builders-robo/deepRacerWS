from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Lidar node
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{'channel_type':'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'}],
    )
    # Hardware node
    hardware = Node(
        package='ros2_pca9685',
        executable='listener',
        name='hardware_interface'
    )
    # Stop logic node
    parallel_parking = Node(
        package='challenges',
        executable='parallel_parking',
        name='parallel_parking'
    )
    #reverse_driving node
    reverse_driving = Node(
        package='challenges',
        executable='reverse_driving',
        name='reverse_driving'
    )

    return LaunchDescription([
        lidar, hardware, reverse_driving,
    ])