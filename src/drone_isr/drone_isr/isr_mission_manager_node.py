#!/usr/bin/env python3
"""
Node ROS2 de gestion de mission ISR.
Phase 3.5: Search & Inspect dynamique.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from drone_isr_msgs.msg import DetectionArray, Alert
from drone_isr.trajectory_generator import (
    ZoneConfig,
    Obstacle,
    compute_strip_width,
    generate_lawnmower,
    compute_potential_field_force,
    adjust_waypoints_potential_field
)


class ISRMissionManagerNode(Node):
    """ISR Mission state machine and waypoint navigation."""

    class State:
        GROUNDED = "GROUNDED"
        TAKEOFF = "TAKEOFF"
        SEARCHING = "SEARCHING"
        AWAITING_COMMAND = "AWAITING_COMMAND"
        INSPECTING = "INSPECTING"
        LAND = "LAND"
        COMPLETE = "COMPLETE"

    def __init__(self):
        super().__init__('isr_mission_manager')

        # --- Paramètres ---
        self.declare_parameter('zone_width', 40.0)
        self.declare_parameter('zone_height', 30.0)
        self.declare_parameter('zone_origin_x', -20.0)
        self.declare_parameter('zone_origin_y', -15.0)
        self.declare_parameter('flight_altitude', 8.0)
        self.declare_parameter('flight_speed_ms', 2.0)
        self.declare_parameter('waypoint_tolerance_m', 1.0)
        self.declare_parameter('hover_on_detection_s', 0.0)
        self.declare_parameter('min_detection_confidence', 0.60)
        self.declare_parameter('dedup_radius_m', 2.0)
        self.declare_parameter('camera_fov_h', 1.047)
        self.declare_parameter('strip_overlap', 0.20)
        self.declare_parameter('max_mission_duration_s', 600.0)
        
        # Obstacles listes plates (dummy par défaut pour éviter Inférence BYTE_ARRAY)
        self.declare_parameter('obstacle_xs', [-999.0])
        self.declare_parameter('obstacle_ys', [-999.0])
        self.declare_parameter('obstacle_zs', [-999.0])
        self.declare_parameter('obstacle_radii', [-999.0])
        self.declare_parameter('obstacle_heights', [-999.0])

        # Extraction paramètres
        self._zone = ZoneConfig(
            width=self.get_parameter('zone_width').value,
            height=self.get_parameter('zone_height').value,
            origin_x=self.get_parameter('zone_origin_x').value,
            origin_y=self.get_parameter('zone_origin_y').value,
            altitude=self.get_parameter('flight_altitude').value,
            strip_width=0.0,
            overlap=self.get_parameter('strip_overlap').value
        )
        self._zone.strip_width = compute_strip_width(
            self._zone.altitude,
            self.get_parameter('camera_fov_h').value,
            self._zone.overlap
        )
        self._flight_altitude = self._zone.altitude
        self._flight_speed = self.get_parameter('flight_speed_ms').value
        self._wp_tolerance = self.get_parameter('waypoint_tolerance_m').value
        self._hover_duration = self.get_parameter('hover_on_detection_s').value
        self._min_confidence = self.get_parameter('min_detection_confidence').value
        self._dedup_radius = self.get_parameter('dedup_radius_m').value
        self._max_mission_duration = self.get_parameter('max_mission_duration_s').value

        # Parsing obstacles depuis les listes plates
        xs = self.get_parameter('obstacle_xs').value
        ys = self.get_parameter('obstacle_ys').value
        zs = self.get_parameter('obstacle_zs').value
        rs = self.get_parameter('obstacle_radii').value
        hs = self.get_parameter('obstacle_heights').value
        
        self._obstacles = []
        if len(xs) > 0 and xs[0] != -999.0:
            for x, y, z, r, h in zip(xs, ys, zs, rs, hs):
                self._obstacles.append(Obstacle(x=x, y=y, z=z, radius=r, height=h))

        # --- État interne ---
        raw_waypoints = generate_lawnmower(self._zone)
        self._waypoints = adjust_waypoints_potential_field(raw_waypoints, self._obstacles)
        
        self._current_wp_idx = 0
        self._state = self.State.GROUNDED
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_z = 0.0
        self._pose_yaw = 0.0
        
        self._seen_targets = []  # liste de tuples (id, tx, ty, label)
        self._next_target_id = 0
        self._target_to_inspect = None # Tuple (tx, ty)
        
        self._ticks = 0
        self._odom_received = False

        # --- Subscribers / Publishers ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.detections_sub = self.create_subscription(DetectionArray, '/detections', self._detections_cb, 10)
        self.cmd_sub = self.create_subscription(String, '/operator_command', self._operator_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alerts_pub = self.create_publisher(Alert, '/alerts', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        self.wp_markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.tgt_markers_pub = self.create_publisher(MarkerArray, '/target_markers', 10)
        self.obs_markers_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self.get_logger().info(f'ISR Mission Manager Init. Lawnmower WPs: {len(self._waypoints)}')
        self.get_logger().info(f'Loaded {len(self._obstacles)} obstacles.')

    def _odom_cb(self, msg: Odometry) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        self._pose_z = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self._odom_received = True

        # Exécuter la boucle de contrôle de la mission directement à la fréquence Odom (50Hz)
        # Cela évite le bug ROS2 de timer figé lors du saut temporel de l'initialisation use_sim_time
        self._control_loop()

    def _operator_cb(self, msg: String) -> None:
        """Parse les ordres opérateurs (ex: 'inspect 0', 'return', 'land')."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"[OPERATOR COMMAND] received: '{cmd}'")
        
        if cmd == "land":
            self.get_logger().warn("Operator requested direct LAND.")
            self._state = self.State.LAND
            
        elif cmd == "return":
            if self._state not in [self.State.GROUNDED, self.State.TAKEOFF, self.State.LAND, self.State.COMPLETE]:
                self.get_logger().info("Returning to Loiter position.")
                self._state = self.State.AWAITING_COMMAND
                
        elif cmd.startswith("inspect "):
            try:
                target_id = int(cmd.split(" ")[1])
                target = next((t for t in self._seen_targets if t[0] == target_id), None)
                if target:
                    self._target_to_inspect = (target[1], target[2])
                    self.get_logger().info(f"Diving to inspect target ID {target_id} ({target[3]})")
                    self._state = self.State.INSPECTING
                else:
                    self.get_logger().error(f"Target ID {target_id} not found.")
            except (ValueError, IndexError):
                self.get_logger().error("Invalid inspect command. Use 'inspect <ID>'.")

    def _detections_cb(self, msg: DetectionArray) -> None:
        if self._state in [self.State.GROUNDED, self.State.TAKEOFF, self.State.LAND, self.State.COMPLETE]:
            return

        for det in msg.detections:
            if det.confidence >= self._min_confidence:
                # Vérifier si c'est un doublon
                is_duplicate = False
                for t in self._seen_targets:
                    dist = math.sqrt((t[1] - det.world_x)**2 + (t[2] - det.world_y)**2)
                    if dist < self._dedup_radius and t[3] == det.label:
                        is_duplicate = True
                        break
                        
                if not is_duplicate:
                    # Nouvelle cible
                    t_id = self._next_target_id
                    self._next_target_id += 1
                    self._seen_targets.append((t_id, det.world_x, det.world_y, det.label))
                    
                    alert = Alert()
                    alert.alert_type = "TARGET_DETECTED"
                    alert.target_label = det.label
                    alert.target_x = det.world_x
                    alert.target_y = det.world_y
                    alert.confidence = det.confidence
                    alert.stamp = self.get_clock().now().to_msg()
                    self.alerts_pub.publish(alert)

                    self.get_logger().warn(
                        f'NEW TARGET [{t_id}]: {det.label} (conf={det.confidence:.2f}) '
                        f'at ({det.world_x:.1f}, {det.world_y:.1f})'
                    )
                    # On ne s'arrête pas, on continue la mission !

    def _control_loop(self) -> None:
        if not self._odom_received:
            return
            
        self._ticks += 1

        msg = String()
        msg.data = self._state
        self.status_pub.publish(msg)
        
        self._publish_rviz_markers()

        if self._state == self.State.GROUNDED:
            # Attend ~3 secondes avant de décoller (Odom(50Hz) = 150 ticks)
            if self._ticks > 150:
                self.get_logger().info('Transition to TAKEOFF')
                self._state = self.State.TAKEOFF
            else:
                self.cmd_vel_pub.publish(Twist())

        elif self._state == self.State.TAKEOFF:
            target_alt = self._flight_altitude * 0.975
            safety_cap = self._flight_altitude * 2.0
            if self._ticks % 50 == 0:
                self.get_logger().info(f'TAKEOFF altitude: {self._pose_z:.2f}m / {target_alt:.1f}m')
            
            if self._pose_z > safety_cap:
                self.get_logger().error('SAFETY: altitude cap reached, forcing SEARCHING')
                self._state = self.State.SEARCHING
                
            elif self._pose_z > target_alt:
                self.get_logger().info('Takeoff complete. Sweeping zone (SEARCHING)...')
                self._state = self.State.SEARCHING
            else:
                twist = Twist()
                twist.linear.z = min(2.0, max(1.0, (target_alt - self._pose_z) * 0.3))
                self.cmd_vel_pub.publish(twist)

        elif self._state == self.State.SEARCHING:
            if self._current_wp_idx >= len(self._waypoints):
                self.get_logger().info('Search pattern complete. Transition to AWAITING_COMMAND.')
                self._state = self.State.AWAITING_COMMAND
                return

            wp = self._waypoints[self._current_wp_idx]
            reached = self._drive_to(wp.x, wp.y, wp.z, self._wp_tolerance, self._flight_speed)
            if reached:
                self.get_logger().info(f'Waypoint {self._current_wp_idx + 1}/{len(self._waypoints)} reached.')
                self._current_wp_idx += 1

        elif self._state == self.State.AWAITING_COMMAND:
            # Loiter au-dessus du centre de la zone pour vue globale
            center_x = self._zone.origin_x + self._zone.width / 2.0
            center_y = self._zone.origin_y + self._zone.height / 2.0
            self._drive_to(center_x, center_y, self._flight_altitude, 0.5, self._flight_speed)

        elif self._state == self.State.INSPECTING:
            # S'approche de la cible ciblée
            if self._target_to_inspect:
                tx, ty = self._target_to_inspect
                self._drive_to(tx, ty, 3.0, 0.5, self._flight_speed)
            else:
                self._state = self.State.AWAITING_COMMAND
                
        elif self._state == self.State.LAND:
            if self._pose_z <= 0.2:
                self.get_logger().info(f'MISSION COMPLETE. Total targets: {len(self._seen_targets)}')
                alert = Alert(alert_type="MISSION_COMPLETE")
                alert.stamp = self.get_clock().now().to_msg()
                self.alerts_pub.publish(alert)
                self.cmd_vel_pub.publish(Twist()) 
                self._state = self.State.COMPLETE
            else:
                twist = Twist()
                twist.linear.z = -1.0
                self.cmd_vel_pub.publish(twist)
                
        elif self._state == self.State.COMPLETE:
            self.cmd_vel_pub.publish(Twist())

    def _drive_to(self, target_x: float, target_y: float, target_z: float, tolerance: float, speed: float) -> bool:
        """Génère la commande twist pour aller à la cible, gère le potentiel d'évitement. Retourne False si en transit, True si atteint."""
        dx = target_x - self._pose_x
        dy = target_y - self._pose_y
        dz = target_z - self._pose_z
        dist_3d = math.hypot(math.hypot(dx, dy), dz)
        dist_xy = math.hypot(dx, dy)

        if dist_3d < tolerance:
            # Nous stabilisons la poussée ("Hover") si on demande de conduire à un point déjà atteint
            twist = Twist()
            # Maintient du Z
            twist.linear.z = min(max(dz * 0.5, -1.0), 1.0)
            self.cmd_vel_pub.publish(twist)
            return True

        twist = Twist()
        
        speed_nom = min(speed, dist_xy * 1.5) # slowdown approach
        vx_nom = 0.0
        vy_nom = 0.0
        if dist_xy > 0.05:
            vx_nom = (dx / dist_xy) * speed_nom
            vy_nom = (dy / dist_xy) * speed_nom
            
        # Potentiel field réactif
        fx, fy, _ = compute_potential_field_force(
            position=(self._pose_x, self._pose_y, self._pose_z),
            obstacles=self._obstacles,
            repulsion_gain=5.0,
            influence_radius=4.0
        )
        
        # Vecteur de vélocité totale (World Frame)
        world_vx = vx_nom + fx
        world_vy = vy_nom + fy
        
        # Rotation vers le repère local du drone (Body Frame)
        cw_yaw = -self._pose_yaw
        local_vx = world_vx * math.cos(cw_yaw) - world_vy * math.sin(cw_yaw)
        local_vy = world_vx * math.sin(cw_yaw) + world_vy * math.cos(cw_yaw)
        
        twist.linear.x = min(max(local_vx, -speed * 1.5), speed * 1.5)
        twist.linear.y = min(max(local_vy, -speed * 1.5), speed * 1.5)
        twist.linear.z = min(max(dz * 0.5, -1.0), 1.0)
        
        if dist_xy > 1.0:
            target_yaw = math.atan2(vy_nom, vx_nom)
            err_yaw = math.atan2(math.sin(target_yaw - self._pose_yaw), math.cos(target_yaw - self._pose_yaw))
            twist.angular.z = max(-1.0, min(err_yaw, 1.0))
        
        self.cmd_vel_pub.publish(twist)
        return False

    def _publish_rviz_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        
        # 1. Waypoints
        wp_array = MarkerArray()
        for idx, wp in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = wp.z
            m.scale.x = m.scale.y = m.scale.z = 0.5
            
            if idx < self._current_wp_idx:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
            elif idx == self._current_wp_idx:
                m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            else:
                m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.5)
            wp_array.markers.append(m)
        self.wp_markers_pub.publish(wp_array)

        # 2. Cibles détectées
        tgt_array = MarkerArray()
        for t_idx, tx, ty, label in self._seen_targets:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.id = 1000 + t_idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = tx
            m.pose.position.y = ty
            m.pose.position.z = 0.5
            m.scale.x = m.scale.y = m.scale.z = 1.5
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            tgt_array.markers.append(m)
            
            t = Marker()
            t.header.frame_id = 'odom'
            t.header.stamp = now
            t.id = 2000 + t_idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.text = f"[{t_idx}] {label}"
            t.pose.position.x = tx
            t.pose.position.y = ty
            t.pose.position.z = 2.0
            t.scale.z = 0.8
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            tgt_array.markers.append(t)
            
        self.tgt_markers_pub.publish(tgt_array)
        
        # 3. Obstacles
        obs_array = MarkerArray()
        for idx, obs in enumerate(self._obstacles):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.id = 3000 + idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = obs.x
            m.pose.position.y = obs.y
            m.pose.position.z = obs.height / 2.0
            diam = (obs.radius + 4.0) * 2.0 
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = obs.height
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.1) # Rouge très transparent
            obs_array.markers.append(m)
            
        self.obs_markers_pub.publish(obs_array)


def main():
    rclpy.init()
    node = ISRMissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        
        try:
            node.destroy_node()
        except Exception:
            pass
            
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

