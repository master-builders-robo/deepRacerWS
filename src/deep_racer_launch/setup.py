import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'deep_racer_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))), # add all launch files
        (os.path.join('share', package_name, 'launch'), [f for f in glob('launch/*.launch.py') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))) # add all config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reverse_driving=challenges.reverse_driving:main',
        ],
    },
)
