# TODO
# Mapping:
#
# Make robot automatically go about scanning env
# Setup https://github.com/SteveMacenski/slam_toolbox
# with odometry from wheel info and lidar data we can generate a map of the environment.

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
    # TODO Install slam_toolbox
    # Node to convert Lidar data into map.
    # https://docs.ros.org/en/humble/p/slam_toolbox/
    mapper = Node(
        package='slam_toolbox',
        executable='',
        name='mapper'
    )
    #target-finding node
    target-finding = Node(
        package='slam_toolbox',
        executable='',
        name='mapper'
    )
    # TODO Create racer logic
    # Node to handle racing logic
    # Uses map to figure out when to turn and how fast to go.
    # racer = Node(
    #     package='racer',
    #     executable='',
    #     name='racer'
    # )
    return LaunchDescription([
        camera, lidar, hardware, mapper
    ])