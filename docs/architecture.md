# Architecture — `drone-isr-ros2`

This document details the complete component and dataflow architecture of the drone-isr-ros2 simulation.

## System Diagram

The following Mermaid diagram outlines the 5 main system groups and their data interactions across the ROS2 and Gazebo Harmonic bridge.

```mermaid
graph TD
    %% Groups
    subgraph Gazebo["Gazebo Sim Harmonic"]
        World[World: surveillance_zone]
        Drone[Model: isr_drone]
        Cam[Camera Sensor 10Hz]
        Imu[IMU Sensor 100Hz]
        
        World --> Drone
        Drone --> Cam
        Drone --> Imu
    end

    subgraph Bridge["ROS-Gz Bridge"]
        ros_gz_bridge[ros_gz_bridge]
        tf_republisher[tf_static_republisher]
    end

    subgraph Perception["Perception Pipeline"]
        perception_node[drone_perception_node]
        yolo_cpu[YOLOv8n CPU]
        
        perception_node <--> yolo_cpu
    end

    subgraph Mission["Mission Control"]
        mission_node[isr_mission_manager_node]
        trajectory_pure[trajectory_generator.py]
        
        mission_node --> trajectory_pure
    end

    subgraph Outputs["Outputs & Visualization"]
        rviz2[rviz2]
        alerts[Logs / External]
    end

    %% Connections Gazebo <-> Bridge
    Cam --> |/camera/image_raw| ros_gz_bridge
    Imu --> |/imu| ros_gz_bridge
    Drone --> |/odom, /tf| ros_gz_bridge
    ros_gz_bridge --> |/cmd_vel| Drone

    %% Connections Bridge <-> ROS2 Nodes
    ros_gz_bridge --> |/camera/image_raw| perception_node
    ros_gz_bridge --> |/tf_static_bridge VOLATILE| tf_republisher
    ros_gz_bridge --> |/odom| mission_node
    mission_node --> |/cmd_vel| ros_gz_bridge

    %% Connections ROS2 Internal
    perception_node --> |/detections| mission_node
    tf_republisher --> |/tf_static TRANSIENT_LOCAL| rviz2
    
    %% Connections Outputs
    perception_node --> |/camera/annotated| rviz2
    mission_node --> |/waypoint_markers, /target_markers| rviz2
    mission_node --> |/alerts, /mission_status| alerts

    %% Styling
    classDef gazebo fill:#f9d0c4,stroke:#333,stroke-width:2px;
    classDef ros fill:#c4e3f3,stroke:#333,stroke-width:2px;
    classDef logic fill:#d4f3c4,stroke:#333,stroke-width:2px;
    
    class World,Drone,Cam,Imu gazebo;
    class ros_gz_bridge,tf_republisher,perception_node,mission_node,rviz2,alerts ros;
    class yolo_cpu,trajectory_pure logic;
```

## Communications (Topics)

| Topic | Type | Hz | Producer | Consumer |
|-------|------|----|----------|----------|
| `/camera/image_raw` | sensor_msgs/Image | 10 | Gazebo | drone_perception_node |
| `/camera/annotated` | sensor_msgs/Image | 3-5 | drone_perception_node | rviz2 |
| `/detections` | drone_isr/DetectionArray | 3-5 | drone_perception_node | isr_mission_manager |
| `/alerts` | drone_isr/Alert | event | isr_mission_manager | log/external |
| `/cmd_vel` | geometry_msgs/Twist | 10 | isr_mission_manager | Gazebo |
| `/odom` | nav_msgs/Odometry | 50 | Gazebo | isr_mission_manager |
| `/tf_static` | tf2_msgs/TFMessage | latched | tf_static_republisher | tf2 + rviz2 |
| `/waypoint_markers` | visualization_msgs/MarkerArray | 1 | isr_mission_manager | rviz2 |
| `/target_markers` | visualization_msgs/MarkerArray | event | isr_mission_manager | rviz2 |
