# Phase 2 — Perception

> **Slash command** : `/phase2`
> **Durée estimée** : Jour 2 (4-6h)
> **Dépendances** : Phase 1 validée (image caméra visible, TF complet)

---

## Objectif

Pipeline de détection d'objets complet : la caméra embarquée du drone détecte les cibles au sol via YOLOv8, publie des messages `Detection` sur `/detections`, et affiche l'overlay visuel (bounding boxes) dans rqt_image_view.

---

## Livrables

1. `msg/Detection.msg` — message custom (bbox, label, confidence, position estimée)
2. `msg/Alert.msg` — message custom pour les alertes mission (Phase 3)
3. `drone_isr/perception_utils.py` — logique PURE : preprocessing, NMS, estimation position 3D
4. `drone_isr/drone_perception_node.py` — node ROS2 : subscribe image → YOLOv8 → publish detections + image annotée
5. `launch/perception.launch.py` — lance uniquement le pipeline de perception
6. `test/test_perception_utils.py` — 10 tests pytest (0 import ROS)

---

## Critères de succès

```bash
# 1. Build + check dépendances YOLOv8
pip3 install ultralytics --break-system-packages
cd ~/ros2_ws
colcon build --symlink-install --packages-select drone_isr
source install/setup.bash

# 2. Tests purs passent (sans Gazebo)
cd ~/ros2_ws/src/drone_isr
python3 -m pytest test/test_perception_utils.py -v
# Résultat attendu : 10/10 PASSED

# 3. Pipeline complet (Gazebo + perception en parallèle)
# Terminal 1
ros2 launch drone_isr simulation.launch.py
# Terminal 2
ros2 launch drone_isr perception.launch.py

# 4. Topics de sortie actifs
ros2 topic echo /detections --once
# Doit afficher un message Detection (même vide si pas de cible visible)

ros2 topic echo /camera/annotated --once
# Image avec bboxes dessinées

# 5. Vérifier les FPS de traitement
ros2 topic hz /detections
# Attendu : ~2-5 Hz sur CPU ARM (acceptable)

# 6. Faire survoler une cible et vérifier détection
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.3}}" --rate 10
# Observer les logs du drone_perception_node : "Detected: car (conf=0.72)"
```

---

## Spécifications techniques

### msg/Detection.msg

```
# Coordonnées bounding box (pixels)
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# Classification
string label
float32 confidence

# Estimation position monde (calculée si altitude connue)
float32 world_x
float32 world_y
float32 world_z

# Timestamp
builtin_interfaces/Time stamp
```

### msg/Alert.msg

```
string alert_type        # "TARGET_DETECTED", "ZONE_COVERED", "MISSION_COMPLETE"
string target_label
float32 target_x
float32 target_y
float32 confidence
builtin_interfaces/Time stamp
```

### perception_utils.py (module PUR)

```python
"""
Logique de perception pure — 0 import ROS2.
Toutes les fonctions sont testables sans ROS sourcé.
"""
from dataclasses import dataclass
import numpy as np
import cv2
from typing import Optional

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

def preprocess_image(image: np.ndarray, target_size: tuple[int, int] = (640, 640)) -> np.ndarray:
    """Redimensionne et normalise l'image pour YOLOv8."""
    ...

def run_detection(model, image: np.ndarray, conf_threshold: float = 0.5) -> list[DetectionResult]:
    """Lance l'inférence YOLOv8 et retourne les détections filtrées."""
    ...

def estimate_world_position(
    bbox_center_x: int,
    bbox_center_y: int,
    image_width: int,
    image_height: int,
    drone_altitude: float,
    drone_pose: tuple[float, float, float],
    camera_fov_h: float = 1.047
) -> tuple[float, float, float]:
    """
    Estime la position monde d'une cible à partir de sa position pixel.
    Suppose que la caméra est orientée vers le bas (nadir).
    Utilise la géométrie sténopé simple.
    """
    ...

def draw_detections(image: np.ndarray, detections: list[DetectionResult]) -> np.ndarray:
    """Dessine les bounding boxes et labels sur l'image (OpenCV)."""
    ...

def filter_low_confidence(
    detections: list[DetectionResult],
    min_confidence: float
) -> list[DetectionResult]:
    """Filtre les détections sous le seuil de confiance."""
    ...
```

### drone_perception_node.py

```python
class DronePerceptionNode(Node):
    """
    Node ROS2 de perception — wrapper fin autour de perception_utils.
    Subscribe : /camera/image_raw (sensor_msgs/Image)
    Publish  : /detections (drone_isr/Detection[])
              /camera/annotated (sensor_msgs/Image)
    """
    def __init__(self):
        super().__init__('drone_perception')
        # Charger les params depuis isr_params.yaml
        self.declare_parameter('model_name', 'yolov8n.pt')  # nano = le plus léger
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', ['car', 'person', 'truck'])
        self.declare_parameter('camera_fov_h', 1.047)

        # Charger le modèle une seule fois au démarrage
        model_name = self.get_parameter('model_name').value
        self.model = YOLO(model_name)

        # Subscribers / Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.detections_pub = self.create_publisher(DetectionArray, '/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated', 10)

        # Subscribe à l'odom pour connaître l'altitude du drone
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._drone_altitude = 1.5  # valeur par défaut

    def _image_cb(self, msg: Image) -> None:
        """Traite une image et publie les détections."""
        image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = run_detection(self.model, image, self._conf_threshold)
        # Estimation position monde pour chaque détection
        # Publication sur /detections et /camera/annotated
        ...
```

### Stratégie YOLOv8 sur CPU ARM

- Utiliser `yolov8n.pt` (nano) — le plus léger, ~3 FPS sur CPU ARM
- Traiter 1 image sur 3 (skip frames) pour ne pas saturer le CPU
- Lazy loading : ne charger le modèle qu'au premier message image reçu
- Paramètre `device: 'cpu'` explicite dans l'appel YOLO

```python
# Dans drone_perception_node.py
results = self.model(image, device='cpu', conf=self._conf_threshold, verbose=False)
```

### Workaround si YOLOv8 trop lent

Si le pipeline est trop lent (< 1 FPS), fallback sur un détecteur OpenCV simple :
```python
# HOG + SVM pour la détection de personnes (intégré OpenCV, pas de téléchargement)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
boxes, _ = hog.detectMultiScale(image, winStride=(8,8))
```

Ce fallback est documenté dans les design_decisions.md comme choix conscient pour les contraintes VM ARM.

### isr_params.yaml

```yaml
drone_perception:
  ros__parameters:
    model_name: "yolov8n.pt"
    confidence_threshold: 0.50
    target_classes: ["car", "person", "truck", "bicycle"]
    camera_fov_h: 1.047         # 60° en radians
    process_every_n_frames: 3   # Skip frames pour économiser CPU
    publish_annotated_image: true
```

---

## test_perception_utils.py — structure des tests

```python
# Tests à implémenter (0 import ROS) :

def test_preprocess_image_resizes_correctly():
def test_preprocess_image_handles_different_inputs():
def test_filter_low_confidence_removes_below_threshold():
def test_filter_low_confidence_keeps_above_threshold():
def test_filter_low_confidence_empty_list():
def test_estimate_world_position_nadir_center():
    # Cible au centre de l'image → position = position du drone (projeté au sol)
def test_estimate_world_position_nadir_offset():
    # Cible décalée de 100px → offset monde cohérent avec l'altitude et le FOV
def test_estimate_world_position_altitude_scaling():
    # Doubler l'altitude → doubler l'offset monde pour même offset pixel
def test_draw_detections_returns_image():
def test_draw_detections_correct_dimensions():
# + 5 tests de détection mock (sans vraie inférence YOLOv8)
```

---

## Fin de phase — checklist avant Phase 3

- [ ] `python3 -m pytest test/test_perception_utils.py -v` : 10/10 PASSED
- [ ] `/detections` publié et structuré correctement
- [ ] Image annotée visible dans rqt_image_view avec bboxes
- [ ] Logs : "Detected: person (conf=0.XX)" quand drone survole une cible
- [ ] FPS de traitement stable (pas de crash OOM)
- [ ] Commit : `git commit -m "feat: Phase 2 - YOLOv8 perception node, custom msgs, position estimation"`
