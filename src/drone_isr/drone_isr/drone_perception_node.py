#!/usr/bin/env python3
"""
Node ROS2 de perception — wrapper fin autour de perception_utils.

Subscribe :
    /camera/image_raw  (sensor_msgs/Image)
    /odom              (nav_msgs/Odometry)

Publish :
    /detections        (drone_isr_msgs/DetectionArray)
    /camera/annotated  (sensor_msgs/Image)

Stratégie de détection (simulation Gazebo) :
  1. Tente de charger YOLOv8. Si import échoue → mode HSV automatique.
  2. En mode HSV, détecte véhicules bleu/rouge/gris et personnes vertes.
  3. En mode YOLO, utilise YOLO (fonctionne en conditions réelles).
  Paramètre 'use_hsv_detection: true' force le mode HSV.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from drone_isr_msgs.msg import Detection, DetectionArray

from drone_isr.perception_utils import (
    DetectionResult,
    draw_detections,
    estimate_world_position,
    run_detection,
    detect_targets_hsv,
)


class DronePerceptionNode(Node):
    """Perception pipeline : camera → YOLOv8 / HSV → detections + annotated image."""

    def __init__(self) -> None:
        super().__init__('drone_perception')

        # --- Parameters ---
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('target_classes', ['car', 'person', 'truck'])
        self.declare_parameter('camera_fov_h', 1.047)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_annotated_image', True)
        # use_hsv_detection=true → force HSV (simulation Gazebo). False → tente YOLO.
        self.declare_parameter('use_hsv_detection', True)

        self._model_name: str = self.get_parameter('model_name').value
        self._conf_threshold: float = self.get_parameter('confidence_threshold').value
        self._target_classes: list[str] = self.get_parameter('target_classes').value
        self._camera_fov_h: float = self.get_parameter('camera_fov_h').value
        self._process_every_n: int = self.get_parameter('process_every_n_frames').value
        self._publish_annotated: bool = self.get_parameter('publish_annotated_image').value
        self._use_hsv: bool = self.get_parameter('use_hsv_detection').value

        # --- Lazy model loading ---
        self._model = None
        self._yolo_unavailable: bool = False   # True si import ultralytics a échoué
        self._frame_count: int = 0
        self._bridge = CvBridge()

        # --- Drone state (from /odom) ---
        self._drone_x: float = 0.0
        self._drone_y: float = 0.0
        self._drone_altitude: float = 1.5  # Valeur par défaut

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
        )

        # --- Publishers ---
        self.detections_pub = self.create_publisher(DetectionArray, '/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated', 10)

        mode_str = 'HSV (simulation)' if self._use_hsv else f'YOLO:{self._model_name}'
        self.get_logger().info(
            f'Perception node started — mode={mode_str}, '
            f'conf={self._conf_threshold}, '
            f'process_every={self._process_every_n}'
        )

    def _load_model(self) -> None:
        """Charge le modèle YOLOv8 au premier appel (lazy loading)."""
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'Loading YOLOv8 model: {self._model_name}...')
            self._model = YOLO(self._model_name)
            self.get_logger().info('YOLOv8 model loaded successfully')
        except ImportError:
            self.get_logger().warn(
                'ultralytics not installed — switching to HSV detection mode.')
            self._yolo_unavailable = True
            self._use_hsv = True
        except Exception as e:
            self.get_logger().warn(f'YOLOv8 load failed ({e}) — switching to HSV detection.')
            self._yolo_unavailable = True
            self._use_hsv = True

    def _odom_cb(self, msg: Odometry) -> None:
        """Met à jour la position et altitude du drone."""
        self._drone_x = msg.pose.pose.position.x
        self._drone_y = msg.pose.pose.position.y
        self._drone_altitude = msg.pose.pose.position.z

    def _image_cb(self, msg: Image) -> None:
        """Traite une image et publie les détections."""
        self._frame_count += 1

        # Frame skipping (même si 1 par défaut sur ARM)
        if self._frame_count % self._process_every_n != 0:
            return

        # Conversion ROS Image → OpenCV BGR
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        image_h, image_w = cv_image.shape[:2]

        # --- Détection : HSV (simulation) ou YOLOv8 (réel) ---
        if self._use_hsv:
            # Mode simulation Gazebo : détection couleur HSV
            detections = detect_targets_hsv(cv_image)
        else:
            # Mode réel : YOLOv8 (lazy load)
            if self._model is None and not self._yolo_unavailable:
                self._load_model()
            if self._model is None:
                # YOLO non disponible — bascule HSV
                detections = detect_targets_hsv(cv_image)
            else:
                detections = run_detection(
                    self._model,
                    cv_image,
                    conf_threshold=self._conf_threshold,
                    target_classes=self._target_classes,
                )

        # --- Estimation position monde ---
        drone_pose = (self._drone_x, self._drone_y, self._drone_altitude)

        for det in detections:
            cx, cy = det.center
            wx, wy, wz = estimate_world_position(
                bbox_center_x=cx,
                bbox_center_y=cy,
                image_width=image_w,
                image_height=image_h,
                drone_altitude=self._drone_altitude,
                drone_pose=drone_pose,
                camera_fov_h=self._camera_fov_h,
            )
            det.world_x = wx
            det.world_y = wy
            det.world_z = wz

        # --- Log détections ---
        for det in detections:
            self.get_logger().info(
                f'Detected: {det.label} (conf={det.confidence:.2f}) '
                f'at world ({det.world_x:.1f}, {det.world_y:.1f})'
            )

        # --- Publish DetectionArray ---
        det_array = DetectionArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = 'camera_link'

        for det in detections:
            det_msg = Detection()
            det_msg.x_min = det.x_min
            det_msg.y_min = det.y_min
            det_msg.x_max = det.x_max
            det_msg.y_max = det.y_max
            det_msg.label = det.label
            det_msg.confidence = det.confidence
            det_msg.world_x = det.world_x
            det_msg.world_y = det.world_y
            det_msg.world_z = det.world_z
            det_msg.stamp = msg.header.stamp
            det_array.detections.append(det_msg)

        self.detections_pub.publish(det_array)

        # --- Publish annotated image ---
        if self._publish_annotated:
            annotated = draw_detections(cv_image, detections)
            try:
                annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')


def main() -> None:
    """Entry point for drone_perception node."""
    rclpy.init()
    node = DronePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
