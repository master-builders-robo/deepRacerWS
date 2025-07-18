from setuptools import setup

package_name = 'ros2_pca9685'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='main',
    maintainer_email='stevej52@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = ros2_pca9685.publisher_member_function:main',
             'listener = ros2_pca9685.subscriber_member_function:main',
             'target_finding=challenges.target_finding:main',
        ],
    },
)
