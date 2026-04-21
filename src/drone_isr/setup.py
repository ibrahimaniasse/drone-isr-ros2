from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_isr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        # Model files — isr_drone
        (os.path.join('share', package_name, 'models', 'isr_drone'),
            glob('models/isr_drone/*')),
        # Model files — gennevilliers OSM terrain
        (os.path.join('share', package_name, 'models', 'gennevilliers'),
            ['models/gennevilliers/model.config']),
        (os.path.join('share', package_name, 'models', 'gennevilliers', 'meshes'),
            glob('models/gennevilliers/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ISR Drone Developer',
    maintainer_email='dev@drone-isr.local',
    description='ISR drone simulation with Gazebo Harmonic and ROS2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_static_republisher = drone_isr.tf_static_republisher:main',
            'drone_perception = drone_isr.drone_perception_node:main',
            'isr_mission_manager = drone_isr.isr_mission_manager_node:main',
            'operator_teleop = drone_isr.operator_teleop:main',
        ],
    },
)
