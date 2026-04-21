#!/usr/bin/env python3
"""
full_mission.launch.py — Phase 3 ISR Drone

Lance toute la mission de bout en bout de manière séquencée :
1. Gazebo + world
2. Param bridge
3. Publish TF statique
4. Node de perception (YOLOv8)
5. RViz2 pour visualisation
6. ISR Mission Manager Node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('drone_isr')

    # --- 1. Simulation (Gazebo + Bridge + TF Statique) ---
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')
        )
    )

    # --- 2. Perception (YOLOv8) ---
    perception_launch = TimerAction(
        period=7.0, # Attendre que gazebo & les bridges soient up
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'perception.launch.py')
                )
            )
        ]
    )

    # --- 3. ISR Mission Manager ---
    mission_params = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    mission_node = TimerAction(
        period=12.0, # Lancer après perception & TF
        actions=[
            Node(
                package='drone_isr',
                executable='isr_mission_manager',
                name='isr_mission_manager',
                parameters=[mission_params, {'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        simulation_launch,
        perception_launch,
        mission_node
    ])
