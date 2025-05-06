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
    # Obstacle Avoidance node
    obstacle_avoidance = Node(
        package='challenges',
        executable='obstacle_avoidance',
        name='obstacle_avoidance'
    )
    return LaunchDescription([
        lidar, obstacle_avoidance, hardware
    ])