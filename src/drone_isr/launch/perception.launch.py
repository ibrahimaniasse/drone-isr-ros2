#!/usr/bin/env python3
"""
perception.launch.py — Phase 2 ISR Drone

Lance uniquement le pipeline de perception YOLOv8.
Prérequis : simulation.launch.py en cours d'exécution.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the perception pipeline launch description."""
    pkg_share = get_package_share_directory('drone_isr')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    perception_node = Node(
        package='drone_isr',
        executable='drone_perception',
        name='drone_perception',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        perception_node,
    ])
