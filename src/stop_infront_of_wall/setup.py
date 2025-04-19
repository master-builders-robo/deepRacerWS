from setuptools import find_packages, setup

package_name = 'stop_infront_of_wall'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher=stop_infront_of_wall.cmd_vel_publisher:main',
            'lidar_subscriber=stop_infront_of_wall.lidar_subscriber:main'
        ],
    },
)
