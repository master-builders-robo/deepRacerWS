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
    # Stop logic node
    lidar_map = Node(
        package='lidar_local_map',
        executable='lidar_local_map',
        name='lidar_local_map'
    )
    return LaunchDescription([
        lidar, lidar_map
    ])