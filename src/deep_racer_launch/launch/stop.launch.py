from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera node
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera'
    )
    # Lidar node
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='lidar'
    )
    # Hardware node
    hardware = Node(
        package='ros2_pca9685',
        executable='listener',
        name='hardware_interface'
    )
    # Stop logic node
    stop = Node(
        package='stop_infront_of_wall',
        executable='',
        name='stop'
    )
    return LaunchDescription([
        camera, lidar, hardware, stop
    ])