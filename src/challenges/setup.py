from setuptools import find_packages, setup

package_name = 'challenges'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'util'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_infront_of_wall=challenges.stop_infront_of_wall:main',
            'parallel_parking=challenges.parallel_parking_two:main',
            'target_finding=challenges.target_finding:main',
        ],
    },
)
