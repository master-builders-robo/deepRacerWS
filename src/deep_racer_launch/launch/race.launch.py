from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera node
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera'
    )
    # Hardware node
    hardware = Node(
        package='ros2_pca9685',
        executable='listener',
        name='hardware_interface'
    )
    # Racer node
    racer = Node(
        package='racer',
        executable='racer',
        name='racer'
    )
    return LaunchDescription([
        camera, hardware, racer
    ])