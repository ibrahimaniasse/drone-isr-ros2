#!/usr/bin/env python3
"""
demo_mission.launch.py — Démo ISR complète, une seule commande

Lance la mission ISR de A à Z :
  1.  0s  — Gazebo Harmonic + surveillance_zone.sdf (drone intégré)
  2. 10s  — ros_gz_bridge (topics caméra, IMU, odométrie, cmd_vel, TF)
  3. 15s  — tf_static_republisher (base_link → camera_link / imu_link / …)
  4. 20s  — drone_perception (mode HSV — détecte véhicules + personnes colorés)
  5. 25s  — isr_mission_manager (machine à états + lawnmower)
  6. 28s  — RViz2 (waypoints, cibles, drone)

Machine à états attendue :
  [ISR Mission] State: GROUNDED -> TAKEOFF
  [ISR Mission] State: TAKEOFF -> SEARCHING
  [ISR Mission] Waypoint 1/20 reached. Zone coverage: 5%
  ...
  [ISR Mission] NEW TARGET [0]: vehicle (conf=0.82) at (12.0, -4.0)
  [ISR Mission] NEW TARGET [1]: person  (conf=0.78) at (5.0, -12.0)
  ...
  [ISR Mission] State: SEARCHING -> AWAITING_COMMAND
  # Envoyer ordre opérateur pour inspection / atterrissage :
  #   ros2 topic pub /operator_command std_msgs/msg/String "data: 'inspect 0'" --once
  #   ros2 topic pub /operator_command std_msgs/msg/String "data: 'land'" --once
  [ISR Mission] State: AWAITING_COMMAND -> INSPECTING (target 0: vehicle)
  [ISR Mission] State: INSPECTING -> AWAITING_COMMAND
  [ISR Mission] State: AWAITING_COMMAND -> LAND
  [ISR Mission] State: LAND -> COMPLETE
  [ISR Mission] Mission complete. 3 targets identified.

Usage :
  ros2 launch drone_isr demo_mission.launch.py
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
    pkg_share = get_package_share_directory('drone_isr')

    # --- Paths ---
    world_path      = os.path.join(pkg_share, 'worlds', 'surveillance_zone.sdf')
    bridge_config   = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_config     = os.path.join(pkg_share, 'config', 'rviz2_config.rviz')
    models_path     = os.path.join(pkg_share, 'models')
    demo_params     = os.path.join(pkg_share, 'config', 'demo_mission_params.yaml')

    # --- Launch arguments ---
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lancer RViz2')
    use_rviz = LaunchConfiguration('use_rviz')

    # --- GZ_SIM_RESOURCE_PATH ---
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = models_path + (':' + existing_gz_path if existing_gz_path else '')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=new_gz_path)

    # ==========================================================
    # 1. Gazebo Harmonic (0s)
    # ==========================================================
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': new_gz_path},
    )

    # ==========================================================
    # 2. ROS ↔ Gz Bridge (10s — ARM/UTM plus lent)
    # ==========================================================
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

    # ==========================================================
    # 3. TF statiques + odom → base_link (15s)
    # ==========================================================
    tf_nodes = TimerAction(
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

    # ==========================================================
    # 4. Perception HSV (20s)
    # ==========================================================
    perception = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='drone_isr',
                executable='drone_perception',
                name='drone_perception',
                parameters=[demo_params, {'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    # ==========================================================
    # 5. ISR Mission Manager (25s)
    # ==========================================================
    mission = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='drone_isr',
                executable='isr_mission_manager',
                name='isr_mission_manager',
                parameters=[demo_params, {'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    # ==========================================================
    # 6. RViz2 (28s, conditionnel)
    # ==========================================================
    rviz = TimerAction(
        period=28.0,
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
        set_gz_resource_path,
        gz_sim,
        bridge,
        tf_nodes,
        perception,
        mission,
        rviz,
    ])
