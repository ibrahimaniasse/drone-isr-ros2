#!/usr/bin/env python3
"""
Publie les transforms statiques du modèle VTOL Airmobi V35.

Le bridge ros_gz Pose_V → TFMessage ne fonctionne pas sur Jazzy/Harmonic
(topic créé mais aucune donnée publiée). Workaround : publier directement
les TFs statiques connues depuis le SDF avec StaticTransformBroadcaster.

Transforms publiées (toutes relatives à base_link) :
  base_link → camera_link    (caméra nadir, pitch 90°)
  base_link → imu_link       (IMU, centre de masse)
  base_link → wing_left/right (ailes)
  base_link → tail_left/right (empennage V)
  base_link → rotor_fl/fr/rl/rr (nacelles VTOL)
  base_link → pusher_prop    (hélice arrière)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert RPY euler angles (radians) to quaternion (x, y, z, w).

    Uses the ZYX convention matching SDF/Gazebo.
    """
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return (x, y, z, w)


# Static transforms from model.sdf — pose format: (x, y, z, roll, pitch, yaw)
# VTOL Airmobi V35 — 11 links
STATIC_TRANSFORMS: list[tuple[str, str, tuple[float, ...]]] = [
    # Camera nadir — sous le fuselage, pointant vers le bas
    ('base_link', 'camera_link', (0.1, 0.0, -0.1, 0.0, 1.5708, 0.0)),
    # IMU — centre de masse
    ('base_link', 'imu_link', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    # Aile gauche
    ('base_link', 'wing_left', (-0.05, 0.38, 0.0, 0.0, 0.0, 0.0)),
    # Aile droite
    ('base_link', 'wing_right', (-0.05, -0.38, 0.0, 0.0, 0.0, 0.0)),
    # Empennage V gauche
    ('base_link', 'tail_left', (-0.58, 0.12, 0.05, 0.0, 0.0, 0.52)),
    # Empennage V droit
    ('base_link', 'tail_right', (-0.58, -0.12, 0.05, 0.0, 0.0, -0.52)),
    # Rotor front-left
    ('base_link', 'rotor_fl', (0.1, 0.65, 0.02, 0.0, 0.0, 0.0)),
    # Rotor front-right
    ('base_link', 'rotor_fr', (0.1, -0.65, 0.02, 0.0, 0.0, 0.0)),
    # Rotor rear-left
    ('base_link', 'rotor_rl', (-0.2, 0.65, 0.02, 0.0, 0.0, 0.0)),
    # Rotor rear-right
    ('base_link', 'rotor_rr', (-0.2, -0.65, 0.02, 0.0, 0.0, 0.0)),
    # Hélice pusher arrière
    ('base_link', 'pusher_prop', (-0.62, 0.0, 0.0, 0.0, 1.5708, 0.0)),
]


class TfStaticPublisher(Node):
    """Publish static transforms for the ISR drone model."""

    def __init__(self) -> None:
        super().__init__('tf_static_republisher')

        self.broadcaster = StaticTransformBroadcaster(self)

        transforms: list[TransformStamped] = []
        now = self.get_clock().now().to_msg()

        for parent, child, pose in STATIC_TRANSFORMS:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child

            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]

            qx, qy, qz, qw = euler_to_quaternion(pose[3], pose[4], pose[5])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            transforms.append(t)
            self.get_logger().info(f'Static TF: {parent} -> {child}')

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} static transforms (TRANSIENT_LOCAL)'
        )


def main() -> None:
    """Entry point for tf_static_republisher node."""
    rclpy.init()
    node = TfStaticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
