"""
Logique de perception pure — 0 import ROS2.

Fonctions de preprocessing, détection YOLOv8, estimation de position monde,
et overlay OpenCV. Testable sans ROS sourcé.
"""

import math
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np


@dataclass
class DetectionResult:
    """Résultat d'une détection unique."""

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
    def center(self) -> tuple[int, int]:
        """Centre du bounding box en pixels."""
        return ((self.x_min + self.x_max) // 2, (self.y_min + self.y_max) // 2)

    @property
    def area(self) -> int:
        """Surface du bounding box en pixels²."""
        return max(0, self.x_max - self.x_min) * max(0, self.y_max - self.y_min)


# Couleurs par classe pour l'overlay (BGR)
CLASS_COLORS: dict[str, tuple[int, int, int]] = {
    "person": (0, 255, 0),  # Vert
    "car": (255, 100, 0),  # Bleu clair
    "truck": (255, 0, 100),  # Violet
    "bicycle": (0, 200, 255),  # Jaune
}
DEFAULT_COLOR: tuple[int, int, int] = (0, 165, 255)  # Orange


def preprocess_image(
    image: np.ndarray,
    target_size: tuple[int, int] = (640, 640),
) -> np.ndarray:
    """Redimensionne l'image pour YOLOv8.

    Args:
        image: Image BGR (H, W, 3) ou grayscale (H, W).
        target_size: Taille cible (width, height).

    Returns:
        Image redimensionnée en BGR (target_H, target_W, 3).
    """
    if image is None or image.size == 0:
        raise ValueError("Input image is empty or None")

    # Convertir grayscale en BGR si nécessaire
    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    resized = cv2.resize(image, target_size, interpolation=cv2.INTER_LINEAR)
    return resized


def run_detection(
    model: Any,
    image: np.ndarray,
    conf_threshold: float = 0.5,
    target_classes: Optional[list[str]] = None,
) -> list[DetectionResult]:
    """Lance l'inférence YOLOv8 et retourne les détections filtrées.

    Args:
        model: Modèle YOLOv8 chargé (ultralytics.YOLO).
        image: Image BGR (H, W, 3).
        conf_threshold: Seuil de confiance minimum.
        target_classes: Classes à conserver (None = toutes).

    Returns:
        Liste de DetectionResult filtrées par confiance et classe.
    """
    results = model(image, device="cpu", conf=conf_threshold, verbose=False)

    detections: list[DetectionResult] = []

    for result in results:
        if result.boxes is None:
            continue

        for box in result.boxes:
            # Extraire les coordonnées xyxy
            coords = box.xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = (
                int(coords[0]),
                int(coords[1]),
                int(coords[2]),
                int(coords[3]),
            )

            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            label = result.names[class_id]

            # Filtrer par classe si spécifié
            if target_classes is not None and label not in target_classes:
                continue

            detections.append(
                DetectionResult(
                    x_min=x_min,
                    y_min=y_min,
                    x_max=x_max,
                    y_max=y_max,
                    label=label,
                    confidence=confidence,
                )
            )

    return detections


def estimate_world_position(
    bbox_center_x: int,
    bbox_center_y: int,
    image_width: int,
    image_height: int,
    drone_altitude: float,
    drone_pose: tuple[float, float, float],
    camera_fov_h: float = 1.047,
) -> tuple[float, float, float]:
    """Estime la position monde d'une cible via géométrie sténopé.

    Hypothèses :
      - Caméra orientée nadir (exactement vers le bas)
      - Sol plat à z=0
      - Pas de rotation de la caméra autour du nadir

    Args:
        bbox_center_x: Centre X du bbox en pixels.
        bbox_center_y: Centre Y du bbox en pixels.
        image_width: Largeur de l'image en pixels.
        image_height: Hauteur de l'image en pixels.
        drone_altitude: Altitude du drone en mètres.
        drone_pose: Position du drone (x, y, z) en mètres.
        camera_fov_h: FOV horizontal de la caméra en radians.

    Returns:
        (world_x, world_y, world_z) — position estimée au sol.
    """
    if drone_altitude <= 0:
        return (drone_pose[0], drone_pose[1], 0.0)

    if image_width <= 0 or image_height <= 0:
        return (drone_pose[0], drone_pose[1], 0.0)

    # Offset normalisé par rapport au centre de l'image [-0.5, 0.5]
    norm_x = (bbox_center_x - image_width / 2.0) / image_width
    norm_y = (bbox_center_y - image_height / 2.0) / image_height

    # Taille projetée au sol (en mètres) basée sur le FOV
    ground_width = 2.0 * drone_altitude * math.tan(camera_fov_h / 2.0)
    aspect_ratio = image_height / image_width
    ground_height = ground_width * aspect_ratio

    # Position monde = position drone + offset projeté
    world_x = drone_pose[0] + norm_x * ground_width
    world_y = drone_pose[1] - norm_y * ground_height  # Y inversé (image vs monde)
    world_z = 0.0  # Au sol

    return (world_x, world_y, world_z)


def draw_detections(
    image: np.ndarray,
    detections: list[DetectionResult],
) -> np.ndarray:
    """Dessine les bounding boxes et labels sur l'image.

    Args:
        image: Image BGR source.
        detections: Liste de détections à dessiner.

    Returns:
        Copie de l'image avec les annotations.
    """
    annotated = image.copy()

    for det in detections:
        color = CLASS_COLORS.get(det.label, DEFAULT_COLOR)

        # Rectangle du bbox
        cv2.rectangle(annotated, (det.x_min, det.y_min), (det.x_max, det.y_max), color, 2)

        # Label avec fond
        label_text = f"{det.label} {det.confidence:.2f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(label_text, font, font_scale, thickness)

        # Fond du label
        cv2.rectangle(
            annotated,
            (det.x_min, det.y_min - text_h - baseline - 4),
            (det.x_min + text_w, det.y_min),
            color,
            -1,
        )

        # Texte
        cv2.putText(
            annotated,
            label_text,
            (det.x_min, det.y_min - baseline - 2),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
        )

        # Position monde si estimée
        if det.world_x != 0.0 or det.world_y != 0.0:
            pos_text = f"({det.world_x:.1f}, {det.world_y:.1f})"
            cv2.putText(
                annotated,
                pos_text,
                (det.x_min, det.y_max + 15),
                font,
                0.4,
                color,
                1,
            )

    return annotated


def filter_low_confidence(
    detections: list[DetectionResult],
    min_confidence: float,
) -> list[DetectionResult]:
    """Filtre les détections sous le seuil de confiance.

    Args:
        detections: Liste de détections.
        min_confidence: Seuil minimum de confiance [0.0, 1.0].

    Returns:
        Liste filtrée (nouvelles instances, pas de mutation).
    """
    return [d for d in detections if d.confidence >= min_confidence]


# ---------------------------------------------------------------------------
# HSV detection — pour simulation Gazebo (remplace YOLOv8 sur les boîtes colorées)
# Couleurs issues de surveillance_zone.sdf / isr_params.yaml :
#   vehicule_bleu  [0.1, 0.3, 0.8] → HSV H≈[200,240]
#   vehicule_rouge [0.8, 0.1, 0.1] → HSV H≈[0,10] + [170,180]
#   vehicule_gris  [0.5, 0.5, 0.5] → HSV S≈0 (gris)
#   personne       [0.1, 0.7, 0.1] → HSV H≈[90,130]
# ---------------------------------------------------------------------------

_HSV_TARGETS = [
    {
        "label": "vehicle",
        "conf": 0.82,
        "ranges": [
            # Bleu
            (np.array([100, 100, 80], dtype=np.uint8), np.array([130, 255, 255], dtype=np.uint8)),
        ],
    },
    {
        "label": "vehicle",
        "conf": 0.85,
        "ranges": [
            # Rouge (bas)
            (np.array([0, 100, 80], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
            # Rouge (haut)
            (np.array([170, 100, 80], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
        ],
    },
    {
        "label": "vehicle",
        "conf": 0.70,
        "ranges": [
            # Gris (faible saturation, valeur moyenne)
            (np.array([0, 0, 90], dtype=np.uint8), np.array([180, 40, 180], dtype=np.uint8)),
        ],
    },
    {
        "label": "person",
        "conf": 0.78,
        "ranges": [
            # Vert (personnes)
            (np.array([40, 80, 60], dtype=np.uint8), np.array([80, 255, 255], dtype=np.uint8)),
        ],
    },
]

_MIN_CONTOUR_AREA = 120  # pixels² — filtre le bruit


def detect_targets_hsv(image: np.ndarray) -> list[DetectionResult]:
    """Détecte les cibles colorées (véhicules + personnes) par segmentation HSV.

    Remplace YOLOv8 pour la simulation Gazebo où les objets sont des boîtes colorées.

    Args:
        image: Image BGR (H, W, 3).

    Returns:
        Liste de DetectionResult (coordonnées pixel, pas encore de position monde).
    """
    if image is None or image.size == 0:
        return []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    results: list[DetectionResult] = []

    for target in _HSV_TARGETS:
        # Construire le masque (union de toutes les plages HSV de ce target)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in target["ranges"]:
            mask |= cv2.inRange(hsv, lo, hi)

        # Morphologie pour combler les trous
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            results.append(
                DetectionResult(
                    x_min=x,
                    y_min=y,
                    x_max=x + w,
                    y_max=y + h,
                    label=target["label"],
                    confidence=target["conf"],
                )
            )

    # Dédupliquer par IoU (même boîte détectée par deux plages de couleur)
    return _nms_detections(results, iou_threshold=0.4)


def _iou(a: DetectionResult, b: DetectionResult) -> float:
    """Intersection-over-Union entre deux bboxes."""
    ix1 = max(a.x_min, b.x_min)
    iy1 = max(a.y_min, b.y_min)
    ix2 = min(a.x_max, b.x_max)
    iy2 = min(a.y_max, b.y_max)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    area_a = (a.x_max - a.x_min) * (a.y_max - a.y_min)
    area_b = (b.x_max - b.x_min) * (b.y_max - b.y_min)
    return inter / (area_a + area_b - inter)


def _nms_detections(
    detections: list[DetectionResult],
    iou_threshold: float = 0.4,
) -> list[DetectionResult]:
    """Greedy NMS — supprime les doublons par IoU."""
    if not detections:
        return []
    sorted_dets = sorted(detections, key=lambda d: d.confidence, reverse=True)
    kept: list[DetectionResult] = []
    for det in sorted_dets:
        if all(_iou(det, k) < iou_threshold for k in kept):
            kept.append(det)
    return kept
