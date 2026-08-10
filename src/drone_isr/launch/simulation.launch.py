#!/usr/bin/env python3
"""
simulation.launch.py — Phase 1 ISR Drone

Ordre de démarrage (timers calibrés UTM/ARM) :
  1. gz sim surveillance_zone.sdf     (drone inclus dans le world, pas de spawn séparé)
  2. ros_gz_bridge                     (bridge.yaml)          @ 10s
  3. tf_static_republisher             (TRANSIENT_LOCAL)      @ 15s
  4. rviz2                             (rviz2_config.rviz)    @ 20s (optionnel)

Corrections vs plan initial :
  - Pas de gz_spawn_entity (Bug #5 — double spawn)
  - Pas de static_transform_publisher odom→base_link (TF dynamique via bridge)
  - Pas de robot_state_publisher (pas de URDF, PosePublisher + relay suffisent)
  - GZ_SIM_RESOURCE_PATH défini pour que le world SDF trouve model://isr_drone
  - IfCondition sur RViz (use_rviz=false pour demo_mission.launch.py)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the Phase 1 simulation launch description."""
    pkg_share = get_package_share_directory('drone_isr')

    # --- Paths ---
    world_path = os.path.join(pkg_share, 'worlds', 'surveillance_zone.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz2_config.rviz')
    models_path = os.path.join(pkg_share, 'models')

    # --- Launch arguments ---
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch rviz2 with pre-configured view',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # --- GZ_SIM_RESOURCE_PATH — obligatoire pour model://isr_drone ---
    # On ajoute le dossier models/ du package au path de recherche Gazebo
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = models_path + (':' + existing_gz_path if existing_gz_path else '')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_path,
    )

    # --- 1. Gazebo Sim Harmonic ---
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', '-v', '3', world_path,
        ],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': new_gz_path},
    )

    # --- 2. ROS ↔ Gz Bridge (10s — ARM/UTM plus lent) ---
    bridge = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                parameters=[{'config_file': bridge_config}, {'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    # --- 3. TF Static Publisher (15s) ---
    # Publie les transforms statiques directement depuis les données SDF.
    # Le bridge Pose_V → TFMessage est cassé sur Jazzy/Harmonic
    # (topic créé mais aucune donnée publiée).
    tf_static_pub = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='drone_isr',
                executable='tf_static_republisher',
                name='tf_static_republisher',
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link'],
                name='tf_base_footprint_to_base_link',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # --- 4. RViz2 (20s, conditionnel) ---
    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ],
    )

    return LaunchDescription([
        use_rviz_arg,
        headless_arg,
        set_gz_resource_path,
        gz_sim,
        bridge,
        tf_static_pub,
        rviz,
    ])
