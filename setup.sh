#!/usr/bin/env bash
# setup.sh — Initialise la structure complète de drone-isr-ros2
# Usage : bash setup.sh
# Lancer depuis la racine du repo cloné sur Mac (dossier partagé).

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="src/drone_isr"

echo "==> Création de la structure du projet dans : $PROJECT_ROOT"

# ── Dossiers principaux ──────────────────────────────────────────────────────

mkdir -p "$PKG/drone_isr"
mkdir -p "$PKG/msg"
mkdir -p "$PKG/test"
mkdir -p "$PKG/config"
mkdir -p "$PKG/launch"
mkdir -p "$PKG/models/isr_drone"
mkdir -p "$PKG/worlds"
mkdir -p ".github/workflows"
mkdir -p ".agent/workflows"
mkdir -p "docs/demo"
mkdir -p "docker"

# ── Package ROS2 ─────────────────────────────────────────────────────────────

cat > "$PKG/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>drone_isr</name>
  <version>1.0.0</version>
  <description>Autonomous ISR drone simulation — ROS2 Jazzy + Gazebo Harmonic + YOLOv8</description>
  <maintainer email="isaiah@example.com">Isaiah Niasse</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>visualization_msgs</depend>
  <depend>std_msgs</depend>
  <depend>ros_gz_bridge</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > "$PKG/setup.py" << 'EOF'
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_isr'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'isr_drone'),
         glob('models/isr_drone/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaiah Niasse',
    maintainer_email='isaiah@example.com',
    description='Autonomous ISR drone simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_perception = drone_isr.drone_perception_node:main',
            'isr_mission_manager = drone_isr.isr_mission_manager_node:main',
            'tf_static_republisher = drone_isr.tf_static_republisher:main',
        ],
    },
)
EOF

cat > "$PKG/setup.cfg" << 'EOF'
[develop]
script_dir=$base/lib/drone_isr
[install]
install_scripts=$base/lib/drone_isr
EOF

mkdir -p "$PKG/resource"
touch "$PKG/resource/drone_isr"

# ── Modules Python (stubs) ───────────────────────────────────────────────────

cat > "$PKG/drone_isr/__init__.py" << 'EOF'
EOF

cat > "$PKG/drone_isr/trajectory_generator.py" << 'EOF'
"""
Générateur de trajectoires pour drone ISR — AUCUN import ROS2.
Toutes les fonctions sont testables sans ROS sourcé.
"""
from dataclasses import dataclass, field
from typing import List
import math


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    heading: float = 0.0


@dataclass
class ZoneConfig:
    width: float
    height: float
    altitude: float
    strip_width: float = 0.0
    overlap: float = 0.2
    origin_x: float = 0.0
    origin_y: float = 0.0
    camera_fov_h: float = 1.047  # 60 degrees

    def __post_init__(self):
        if self.strip_width == 0.0:
            # Calcul automatique depuis altitude et FOV
            self.strip_width = (
                2 * self.altitude * math.tan(self.camera_fov_h / 2) * (1 - self.overlap)
            )


def generate_lawnmower(config: ZoneConfig) -> List[Waypoint]:
    """Génère un pattern lawnmower (boustrophédon) couvrant toute la zone."""
    # TODO : implémenter par l'agent
    raise NotImplementedError


def generate_circular(
    center_x: float,
    center_y: float,
    radius: float,
    altitude: float,
    n_points: int = 16,
) -> List[Waypoint]:
    """Génère une trajectoire circulaire autour d'un point d'intérêt."""
    # TODO : implémenter par l'agent
    raise NotImplementedError


def compute_total_distance(waypoints: List[Waypoint]) -> float:
    """Calcule la distance totale de la trajectoire."""
    # TODO : implémenter par l'agent
    raise NotImplementedError


def estimate_mission_duration(waypoints: List[Waypoint], speed_ms: float) -> float:
    """Estime la durée de la mission en secondes."""
    if speed_ms <= 0:
        raise ValueError("speed_ms must be positive")
    return compute_total_distance(waypoints) / speed_ms


def filter_waypoints_outside_zone(
    waypoints: List[Waypoint], zone: ZoneConfig
) -> List[Waypoint]:
    """Supprime les waypoints hors de la zone définie."""
    # TODO : implémenter par l'agent
    raise NotImplementedError
EOF

cat > "$PKG/drone_isr/perception_utils.py" << 'EOF'
"""
Utilitaires de perception pour drone ISR — AUCUN import ROS2.
Toutes les fonctions sont testables sans ROS sourcé.
"""
from dataclasses import dataclass
from typing import List, Tuple
import math
import numpy as np


@dataclass
class DetectionResult:
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    label: str
    confidence: float
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0

    @property
    def center_x(self) -> int:
        return (self.x_min + self.x_max) // 2

    @property
    def center_y(self) -> int:
        return (self.y_min + self.y_max) // 2


def filter_low_confidence(
    detections: List[DetectionResult], min_confidence: float
) -> List[DetectionResult]:
    """Filtre les détections sous le seuil de confiance."""
    return [d for d in detections if d.confidence >= min_confidence]


def estimate_world_position(
    bbox_center_x: int,
    bbox_center_y: int,
    image_width: int,
    image_height: int,
    drone_altitude: float,
    drone_pose: Tuple[float, float, float],
    camera_fov_h: float = 1.047,
) -> Tuple[float, float, float]:
    """
    Estime la position monde d'une cible depuis sa position pixel.
    Hypothèse : caméra orientée vers le bas (nadir), pas de roll/pitch.
    """
    # TODO : implémenter par l'agent
    raise NotImplementedError


def draw_detections(
    image: np.ndarray, detections: List[DetectionResult]
) -> np.ndarray:
    """Dessine les bounding boxes et labels sur l'image (OpenCV)."""
    # TODO : implémenter par l'agent
    raise NotImplementedError
EOF

cat > "$PKG/drone_isr/tf_static_republisher.py" << 'EOF'
#!/usr/bin/env python3
"""
Relay /tf_static_bridge (VOLATILE) vers /tf_static (TRANSIENT_LOCAL).
Obligatoire pour Gazebo Harmonic dont le bridge publie en VOLATILE.
tf2_ros exige TRANSIENT_LOCAL pour les transforms statiques.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        volatile_qos = QoSProfile(depth=10)
        transient_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(
            TFMessage, '/tf_static_bridge', self._cb, volatile_qos
        )
        self.pub = self.create_publisher(TFMessage, '/tf_static', transient_qos)
        self.get_logger().info(
            'Bridging /tf_static_bridge (VOLATILE) -> /tf_static (TRANSIENT_LOCAL)'
        )

    def _cb(self, msg: TFMessage) -> None:
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TfStaticRepublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

cat > "$PKG/drone_isr/drone_perception_node.py" << 'EOF'
#!/usr/bin/env python3
"""
Node ROS2 de perception drone ISR.
Subscribe : /camera/image_raw
Publish   : /detections, /camera/annotated
"""
# TODO : implémenter par l'agent en Phase 2
import rclpy
from rclpy.node import Node


class DronePerceptionNode(Node):
    def __init__(self):
        super().__init__('drone_perception')
        self.get_logger().info('DronePerceptionNode started — stub, implement in Phase 2')


def main():
    rclpy.init()
    rclpy.spin(DronePerceptionNode())
    rclpy.shutdown()
EOF

cat > "$PKG/drone_isr/isr_mission_manager_node.py" << 'EOF'
#!/usr/bin/env python3
"""
Node ROS2 gestionnaire de mission ISR.
Subscribe : /detections, /odom
Publish   : /cmd_vel, /alerts, /mission_status, /waypoint_markers, /target_markers
"""
# TODO : implémenter par l'agent en Phase 3
import rclpy
from rclpy.node import Node


class ISRMissionManagerNode(Node):
    def __init__(self):
        super().__init__('isr_mission_manager')
        self.get_logger().info('ISRMissionManagerNode started — stub, implement in Phase 3')


def main():
    rclpy.init()
    rclpy.spin(ISRMissionManagerNode())
    rclpy.shutdown()
EOF

# ── Messages custom ──────────────────────────────────────────────────────────

cat > "$PKG/msg/Detection.msg" << 'EOF'
# Bounding box (pixels)
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# Classification
string label
float32 confidence

# Position monde estimée (géométrie sténopé, caméra nadir)
float32 world_x
float32 world_y
float32 world_z

# Timestamp
builtin_interfaces/Time stamp
EOF

cat > "$PKG/msg/DetectionArray.msg" << 'EOF'
std_msgs/Header header
drone_isr/Detection[] detections
EOF

cat > "$PKG/msg/Alert.msg" << 'EOF'
# Types : TARGET_DETECTED | ZONE_COVERED | MISSION_COMPLETE
string alert_type
string target_label
float32 target_x
float32 target_y
float32 confidence
builtin_interfaces/Time stamp
EOF

# ── Tests (stubs) ────────────────────────────────────────────────────────────

cat > "$PKG/test/test_trajectory_generator.py" << 'EOF'
"""
Tests unitaires pour trajectory_generator.py — 0 import ROS2.
Lancer avec : python3 -m pytest test/test_trajectory_generator.py -v
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from drone_isr.trajectory_generator import (
    Waypoint, ZoneConfig, generate_lawnmower,
    compute_total_distance, estimate_mission_duration,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def small_zone():
    return ZoneConfig(width=20.0, height=15.0, altitude=8.0)


# ── Lawnmower ─────────────────────────────────────────────────────────────────

def test_lawnmower_returns_list(small_zone):
    wps = generate_lawnmower(small_zone)
    assert isinstance(wps, list)


def test_lawnmower_at_least_two_waypoints(small_zone):
    wps = generate_lawnmower(small_zone)
    assert len(wps) >= 2


def test_lawnmower_waypoints_at_correct_altitude(small_zone):
    wps = generate_lawnmower(small_zone)
    for wp in wps:
        assert wp.z == pytest.approx(small_zone.altitude, abs=0.01)


def test_lawnmower_no_duplicate_waypoints(small_zone):
    wps = generate_lawnmower(small_zone)
    positions = [(round(w.x, 2), round(w.y, 2)) for w in wps]
    assert len(positions) == len(set(positions))


def test_lawnmower_alternates_direction(small_zone):
    """Les passages pairs vont dans un sens, impairs dans l'autre."""
    wps = generate_lawnmower(small_zone)
    # TODO : vérifier l'alternance selon l'implémentation agent
    assert len(wps) > 0


def test_lawnmower_covers_full_zone(small_zone):
    """Chaque point de la zone doit être à max strip_width/2 d'un waypoint."""
    wps = generate_lawnmower(small_zone)
    half_strip = small_zone.strip_width / 2
    # Vérifier quelques points représentatifs
    test_points = [(0, 0), (10, 7), (-8, -5), (9, -6)]
    for tx, ty in test_points:
        distances = [
            ((wp.x - tx) ** 2 + (wp.y - ty) ** 2) ** 0.5
            for wp in wps
        ]
        assert min(distances) <= half_strip + 0.5


# ── Distance & durée ──────────────────────────────────────────────────────────

def test_compute_total_distance_zero_for_single_point():
    wps = [Waypoint(0, 0, 8)]
    assert compute_total_distance(wps) == pytest.approx(0.0)


def test_compute_total_distance_simple():
    wps = [Waypoint(0, 0, 8), Waypoint(3, 4, 8)]
    assert compute_total_distance(wps) == pytest.approx(5.0, abs=0.01)


def test_compute_total_distance_empty():
    assert compute_total_distance([]) == pytest.approx(0.0)


def test_estimate_duration_consistent_with_distance():
    wps = [Waypoint(0, 0, 8), Waypoint(10, 0, 8)]
    duration = estimate_mission_duration(wps, speed_ms=2.0)
    assert duration == pytest.approx(5.0, abs=0.01)


def test_estimate_duration_raises_on_zero_speed():
    wps = [Waypoint(0, 0, 8), Waypoint(5, 0, 8)]
    with pytest.raises(ValueError):
        estimate_mission_duration(wps, speed_ms=0.0)


# ── ZoneConfig ────────────────────────────────────────────────────────────────

def test_zone_config_auto_strip_width():
    zone = ZoneConfig(width=40, height=30, altitude=8.0)
    assert zone.strip_width > 0


def test_zone_config_strip_width_increases_with_altitude():
    zone_low = ZoneConfig(width=40, height=30, altitude=5.0)
    zone_high = ZoneConfig(width=40, height=30, altitude=10.0)
    assert zone_high.strip_width > zone_low.strip_width
EOF

cat > "$PKG/test/test_perception_utils.py" << 'EOF'
"""
Tests unitaires pour perception_utils.py — 0 import ROS2.
Lancer avec : python3 -m pytest test/test_perception_utils.py -v
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import numpy as np
from drone_isr.perception_utils import (
    DetectionResult, filter_low_confidence,
    estimate_world_position, draw_detections,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def sample_detections():
    return [
        DetectionResult(10, 20, 100, 200, 'car', 0.85),
        DetectionResult(50, 60, 150, 250, 'person', 0.42),
        DetectionResult(200, 100, 300, 250, 'truck', 0.91),
    ]


@pytest.fixture
def blank_image():
    return np.zeros((480, 640, 3), dtype=np.uint8)


# ── filter_low_confidence ─────────────────────────────────────────────────────

def test_filter_removes_below_threshold(sample_detections):
    result = filter_low_confidence(sample_detections, min_confidence=0.5)
    assert all(d.confidence >= 0.5 for d in result)


def test_filter_keeps_above_threshold(sample_detections):
    result = filter_low_confidence(sample_detections, min_confidence=0.5)
    assert len(result) == 2


def test_filter_empty_list():
    assert filter_low_confidence([], min_confidence=0.5) == []


def test_filter_threshold_zero_keeps_all(sample_detections):
    result = filter_low_confidence(sample_detections, min_confidence=0.0)
    assert len(result) == len(sample_detections)


def test_filter_threshold_one_keeps_none(sample_detections):
    result = filter_low_confidence(sample_detections, min_confidence=1.0)
    assert len(result) == 0


# ── DetectionResult properties ────────────────────────────────────────────────

def test_detection_center_x():
    d = DetectionResult(0, 0, 100, 200, 'car', 0.9)
    assert d.center_x == 50


def test_detection_center_y():
    d = DetectionResult(0, 0, 100, 200, 'car', 0.9)
    assert d.center_y == 100


# ── estimate_world_position ───────────────────────────────────────────────────

def test_estimate_world_position_center_pixel():
    """Cible au centre de l'image -> offset monde nul."""
    wx, wy, wz = estimate_world_position(
        bbox_center_x=320, bbox_center_y=240,
        image_width=640, image_height=480,
        drone_altitude=8.0,
        drone_pose=(0.0, 0.0, 8.0),
    )
    assert wx == pytest.approx(0.0, abs=0.5)
    assert wy == pytest.approx(0.0, abs=0.5)


def test_estimate_world_position_altitude_scaling():
    """Doubler l'altitude double l'offset monde pour même offset pixel."""
    wx_low, wy_low, _ = estimate_world_position(
        320 + 100, 240, 640, 480, 5.0, (0, 0, 5.0))
    wx_high, wy_high, _ = estimate_world_position(
        320 + 100, 240, 640, 480, 10.0, (0, 0, 10.0))
    assert abs(wx_high) > abs(wx_low)


# ── draw_detections ───────────────────────────────────────────────────────────

def test_draw_detections_returns_image(blank_image, sample_detections):
    result = draw_detections(blank_image, sample_detections)
    assert isinstance(result, np.ndarray)


def test_draw_detections_correct_dimensions(blank_image, sample_detections):
    result = draw_detections(blank_image, sample_detections)
    assert result.shape == blank_image.shape


def test_draw_detections_empty_list(blank_image):
    result = draw_detections(blank_image, [])
    assert result.shape == blank_image.shape
EOF

# ── Config ───────────────────────────────────────────────────────────────────

cat > "$PKG/config/isr_params.yaml" << 'EOF'
drone_perception:
  ros__parameters:
    model_name: "yolov8n.pt"
    confidence_threshold: 0.50
    target_classes: ["car", "person", "truck", "bicycle"]
    camera_fov_h: 1.047
    process_every_n_frames: 3
    publish_annotated_image: true

isr_mission_manager:
  ros__parameters:
    zone_width: 40.0
    zone_height: 30.0
    zone_origin_x: -20.0
    zone_origin_y: -15.0
    flight_altitude: 8.0
    flight_speed_ms: 2.0
    waypoint_tolerance_m: 1.0
    hover_on_detection_s: 3.0
    min_detection_confidence: 0.60
    dedup_radius_m: 2.0
    camera_fov_h: 1.047
    strip_overlap: 0.20
    max_mission_duration_s: 300
EOF

# ── Launch files (stubs) ─────────────────────────────────────────────────────

cat > "$PKG/launch/simulation.launch.py" << 'EOF'
"""Launch Phase 1 : Gazebo + bridge + spawn + TF + rviz2."""
# TODO : implémenter par l'agent en Phase 1
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
EOF

cat > "$PKG/launch/perception.launch.py" << 'EOF'
"""Launch Phase 2 : drone_perception_node seul."""
# TODO : implémenter par l'agent en Phase 2
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
EOF

cat > "$PKG/launch/full_mission.launch.py" << 'EOF'
"""Launch Phase 3 : simulation + perception + mission manager."""
# TODO : implémenter par l'agent en Phase 3
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
EOF

# ── Models (stubs) ───────────────────────────────────────────────────────────

cat > "$PKG/models/isr_drone/model.config" << 'EOF'
<?xml version="1.0"?>
<model>
  <name>isr_drone</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Isaiah Niasse</name>
  </author>
  <description>ISR quadrotor drone with downward-facing camera</description>
</model>
EOF

touch "$PKG/models/isr_drone/model.sdf"
touch "$PKG/worlds/surveillance_zone.sdf"
touch "$PKG/config/bridge.yaml"

# ── CI ───────────────────────────────────────────────────────────────────────

cat > ".github/workflows/ci.yml" << 'EOF'
name: CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  lint:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v4
        with:
          python-version: "3.12"
      - run: pip install flake8 black isort
      - run: flake8 src/drone_isr/drone_isr/ --max-line-length=100
      - run: black --check src/drone_isr/drone_isr/
      - run: isort --check-only src/drone_isr/drone_isr/

  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v4
        with:
          python-version: "3.12"
      - run: pip install pytest numpy opencv-python-headless
      - run: python3 -m pytest src/drone_isr/test/ -v --tb=short

  build-docker:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: docker build -f docker/Dockerfile -t drone-isr-ros2:test .
        continue-on-error: true
EOF

# ── Docker stub ──────────────────────────────────────────────────────────────

cat > "docker/Dockerfile" << 'EOF'
# Dockerfile pour recruteurs x86 — NE PAS utiliser pour la simulation sur ARM
FROM ros:humble-ros-base-jammy

WORKDIR /ros2_ws
COPY src/ src/

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install ultralytics opencv-python-headless

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select drone_isr

CMD ["/bin/bash", "-c", ". /opt/ros/humble/setup.sh && . install/setup.sh && ros2 launch drone_isr full_mission.launch.py"]
EOF

# ── README stub ──────────────────────────────────────────────────────────────

cat > "README.md" << 'EOF'
# drone-isr-ros2

> Autonomous ISR drone simulation with object detection — ROS2 Jazzy + Gazebo Harmonic + YOLOv8

<!-- Badges — à remplir après Phase 4 -->
![CI](https://github.com/USERNAME/drone-isr-ros2/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.12-blue)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)

<!-- GIF démo — à ajouter en Phase 4 -->
<!-- ![Demo](docs/demo/demo.gif) -->

## What it does

A simulated ISR drone autonomously surveys a 40x30m zone using a lawnmower coverage pattern,
detects ground targets (vehicles, persons) with YOLOv8, and publishes georeferenced alerts.

## Status

- Phase 1 — Fondations : 🔄 en cours
- Phase 2 — Perception : ⏳ à faire
- Phase 3 — Mission ISR : ⏳ à faire
- Phase 4 — Documentation : ⏳ à faire

## Quick start

```bash
git clone https://github.com/USERNAME/drone-isr-ros2
mkdir -p ~/ros2_ws && ln -s $(pwd)/src ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select drone_isr
source install/setup.bash
ros2 launch drone_isr full_mission.launch.py
```
EOF

# ── .gitignore ───────────────────────────────────────────────────────────────

cat > ".gitignore" << 'EOF'
# ROS2 build artifacts — NE JAMAIS committer
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
.pytest_cache/

# YOLOv8 weights (trop lourds pour git)
*.pt
*.onnx

# macOS
.DS_Store

# IDE
.vscode/
.idea/
EOF

# ── Résumé ───────────────────────────────────────────────────────────────────

echo ""
echo "✅ Structure créée avec succès !"
echo ""
echo "Prochaines étapes :"
echo "  1. git init && git add . && git commit -m 'chore: initial project structure'"
echo "  2. Créer le repo GitHub drone-isr-ros2 et pousser"
echo "  3. Setup workspace VM :"
echo "     mkdir -p ~/ros2_ws"
echo "     ln -s /mnt/mac-share/drone-isr-ros2/src ~/ros2_ws/src"
echo "  4. Lancer /phase1 dans Antigravity"
echo ""
echo "Structure créée :"
find . -not -path './.git/*' -not -path './node_modules/*' | sort | head -60
