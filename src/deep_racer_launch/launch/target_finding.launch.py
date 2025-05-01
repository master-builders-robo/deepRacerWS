from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen'
    )
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
    )
    hardware = Node(
        package='ros2_pca9685',
        executable='listener',
        name='hardware_interface',
        output='screen'
    )
    target_finding = Node(
        package='challenges',
        executable='target_finding',
        name='target_finding',
        output='screen'
    )

    return LaunchDescription([camera, lidar, hardware, target_finding])
