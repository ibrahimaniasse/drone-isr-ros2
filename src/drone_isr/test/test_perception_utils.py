"""
Tests pour perception_utils.py — 0 import ROS.
Exécutable avec : python3 -m pytest test/test_perception_utils.py -v
"""
import math
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest

from drone_isr.perception_utils import (
    DetectionResult,
    draw_detections,
    estimate_world_position,
    filter_low_confidence,
    preprocess_image,
    run_detection,
)


# ============================================================
# DetectionResult dataclass
# ============================================================

class TestDetectionResult:
    """Tests pour le dataclass DetectionResult."""

    def test_creation_with_defaults(self) -> None:
        """Un DetectionResult doit être créable avec les champs obligatoires."""
        det = DetectionResult(
            x_min=10, y_min=20, x_max=100, y_max=200,
            label='car', confidence=0.85,
        )
        assert det.label == 'car'
        assert det.confidence == 0.85
        assert det.world_x == 0.0
        assert det.world_y == 0.0

    def test_center_property(self) -> None:
        """Le centre doit être la moyenne des coins."""
        det = DetectionResult(
            x_min=0, y_min=0, x_max=100, y_max=200,
            label='person', confidence=0.9,
        )
        assert det.center == (50, 100)

    def test_area_property(self) -> None:
        """La surface doit être width * height."""
        det = DetectionResult(
            x_min=10, y_min=20, x_max=110, y_max=220,
            label='truck', confidence=0.7,
        )
        assert det.area == 100 * 200


# ============================================================
# preprocess_image
# ============================================================

class TestPreprocessImage:
    """Tests pour preprocess_image."""

    def test_resizes_correctly(self) -> None:
        """L'image doit être redimensionnée à la taille cible."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = preprocess_image(image, target_size=(320, 320))
        assert result.shape == (320, 320, 3)

    def test_handles_grayscale(self) -> None:
        """Une image grayscale doit être convertie en BGR."""
        image = np.zeros((100, 100), dtype=np.uint8)
        result = preprocess_image(image, target_size=(50, 50))
        assert result.shape == (50, 50, 3)

    def test_handles_rgba(self) -> None:
        """Une image RGBA doit être convertie en BGR."""
        image = np.zeros((100, 100, 4), dtype=np.uint8)
        result = preprocess_image(image, target_size=(50, 50))
        assert result.shape == (50, 50, 3)

    def test_raises_on_empty_image(self) -> None:
        """Doit lever ValueError pour une image vide."""
        with pytest.raises(ValueError, match="empty or None"):
            preprocess_image(np.array([]))

    def test_raises_on_none(self) -> None:
        """Doit lever ValueError pour None."""
        with pytest.raises(ValueError, match="empty or None"):
            preprocess_image(None)


# ============================================================
# filter_low_confidence
# ============================================================

class TestFilterLowConfidence:
    """Tests pour filter_low_confidence."""

    def test_removes_below_threshold(self) -> None:
        """Les détections sous le seuil doivent être retirées."""
        detections = [
            DetectionResult(0, 0, 10, 10, 'car', 0.3),
            DetectionResult(0, 0, 10, 10, 'person', 0.8),
        ]
        result = filter_low_confidence(detections, min_confidence=0.5)
        assert len(result) == 1
        assert result[0].label == 'person'

    def test_keeps_above_threshold(self) -> None:
        """Les détections au-dessus du seuil doivent être conservées."""
        detections = [
            DetectionResult(0, 0, 10, 10, 'car', 0.9),
            DetectionResult(0, 0, 10, 10, 'person', 0.7),
        ]
        result = filter_low_confidence(detections, min_confidence=0.5)
        assert len(result) == 2

    def test_keeps_exact_threshold(self) -> None:
        """Une détection exactement au seuil doit être conservée."""
        detections = [
            DetectionResult(0, 0, 10, 10, 'car', 0.5),
        ]
        result = filter_low_confidence(detections, min_confidence=0.5)
        assert len(result) == 1

    def test_empty_list(self) -> None:
        """Une liste vide doit retourner une liste vide."""
        result = filter_low_confidence([], min_confidence=0.5)
        assert result == []


# ============================================================
# estimate_world_position
# ============================================================

class TestEstimateWorldPosition:
    """Tests pour estimate_world_position."""

    def test_nadir_center(self) -> None:
        """Une cible au centre image → position = position du drone au sol."""
        wx, wy, wz = estimate_world_position(
            bbox_center_x=320, bbox_center_y=240,
            image_width=640, image_height=480,
            drone_altitude=10.0,
            drone_pose=(5.0, 3.0, 10.0),
            camera_fov_h=1.047,
        )
        assert abs(wx - 5.0) < 0.01
        assert abs(wy - 3.0) < 0.01
        assert wz == 0.0

    def test_nadir_offset(self) -> None:
        """Cible décalée → offset monde cohérent."""
        # Pixel au bord droit (x=640, centre=320) → décalage positif en X
        wx, wy, _ = estimate_world_position(
            bbox_center_x=640, bbox_center_y=240,
            image_width=640, image_height=480,
            drone_altitude=10.0,
            drone_pose=(0.0, 0.0, 10.0),
            camera_fov_h=1.047,
        )
        # À 10m d'altitude avec FOV 60°, le bord droit = +ground_width/2
        ground_half_width = 10.0 * math.tan(1.047 / 2.0)
        assert abs(wx - ground_half_width) < 0.1
        assert abs(wy) < 0.01

    def test_altitude_scaling(self) -> None:
        """Doubler l'altitude → double l'offset monde."""
        _, _, _ = estimate_world_position(
            bbox_center_x=480, bbox_center_y=240,
            image_width=640, image_height=480,
            drone_altitude=5.0,
            drone_pose=(0.0, 0.0, 5.0),
            camera_fov_h=1.047,
        )
        wx_5m = estimate_world_position(
            bbox_center_x=480, bbox_center_y=240,
            image_width=640, image_height=480,
            drone_altitude=5.0,
            drone_pose=(0.0, 0.0, 5.0),
            camera_fov_h=1.047,
        )[0]
        wx_10m = estimate_world_position(
            bbox_center_x=480, bbox_center_y=240,
            image_width=640, image_height=480,
            drone_altitude=10.0,
            drone_pose=(0.0, 0.0, 10.0),
            camera_fov_h=1.047,
        )[0]
        assert abs(wx_10m / wx_5m - 2.0) < 0.01

    def test_zero_altitude_returns_drone_pos(self) -> None:
        """Altitude 0 → retourne la position du drone."""
        wx, wy, wz = estimate_world_position(
            bbox_center_x=100, bbox_center_y=100,
            image_width=640, image_height=480,
            drone_altitude=0.0,
            drone_pose=(7.0, 3.0, 0.0),
            camera_fov_h=1.047,
        )
        assert wx == 7.0
        assert wy == 3.0


# ============================================================
# draw_detections
# ============================================================

class TestDrawDetections:
    """Tests pour draw_detections."""

    def test_returns_image(self) -> None:
        """Doit retourner une image numpy."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = [
            DetectionResult(10, 20, 100, 200, 'car', 0.85),
        ]
        result = draw_detections(image, detections)
        assert isinstance(result, np.ndarray)

    def test_correct_dimensions(self) -> None:
        """L'image annotée doit garder les mêmes dimensions."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = [
            DetectionResult(10, 20, 100, 200, 'person', 0.9),
        ]
        result = draw_detections(image, detections)
        assert result.shape == image.shape

    def test_does_not_mutate_original(self) -> None:
        """L'image originale ne doit pas être modifiée."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        original_copy = image.copy()
        detections = [
            DetectionResult(10, 20, 100, 200, 'car', 0.85),
        ]
        draw_detections(image, detections)
        np.testing.assert_array_equal(image, original_copy)

    def test_empty_detections(self) -> None:
        """Aucune détection → image inchangée (copie)."""
        image = np.ones((100, 100, 3), dtype=np.uint8) * 128
        result = draw_detections(image, [])
        np.testing.assert_array_equal(result, image)


# ============================================================
# run_detection (mock model)
# ============================================================

class TestRunDetection:
    """Tests pour run_detection avec un modèle mocké."""

    def _make_mock_model(self, detections: list[dict]) -> MagicMock:
        """Crée un modèle YOLOv8 mocké retournant les détections spécifiées."""
        mock_model = MagicMock()

        mock_results = []
        mock_result = MagicMock()

        if detections:
            mock_boxes = []
            names = {}

            for i, det in enumerate(detections):
                mock_box = MagicMock()
                mock_box.xyxy = [
                    MagicMock(cpu=MagicMock(return_value=MagicMock(
                        numpy=MagicMock(return_value=np.array([
                            det['x_min'], det['y_min'], det['x_max'], det['y_max']
                        ]))
                    )))
                ]
                mock_box.conf = [
                    MagicMock(cpu=MagicMock(return_value=MagicMock(
                        numpy=MagicMock(return_value=np.float32(det['confidence']))
                    )))
                ]
                mock_box.cls = [
                    MagicMock(cpu=MagicMock(return_value=MagicMock(
                        numpy=MagicMock(return_value=np.int32(i))
                    )))
                ]
                mock_boxes.append(mock_box)
                names[i] = det['label']

            mock_result.boxes = mock_boxes
            mock_result.names = names
        else:
            mock_result.boxes = None

        mock_results.append(mock_result)
        mock_model.return_value = mock_results

        return mock_model

    def test_returns_list(self) -> None:
        """run_detection doit retourner une liste."""
        model = self._make_mock_model([])
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = run_detection(model, image)
        assert isinstance(result, list)

    def test_returns_detections(self) -> None:
        """Les détections du modèle doivent être converties en DetectionResult."""
        model = self._make_mock_model([
            {'x_min': 10, 'y_min': 20, 'x_max': 100, 'y_max': 200,
             'label': 'car', 'confidence': 0.85},
        ])
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = run_detection(model, image, conf_threshold=0.5)
        assert len(result) == 1
        assert result[0].label == 'car'
        assert result[0].confidence == pytest.approx(0.85, abs=0.01)

    def test_filters_by_target_classes(self) -> None:
        """Seules les classes cibles doivent être retournées."""
        model = self._make_mock_model([
            {'x_min': 10, 'y_min': 20, 'x_max': 100, 'y_max': 200,
             'label': 'car', 'confidence': 0.85},
            {'x_min': 50, 'y_min': 60, 'x_max': 150, 'y_max': 260,
             'label': 'dog', 'confidence': 0.90},
        ])
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = run_detection(model, image, target_classes=['car', 'person'])
        assert len(result) == 1
        assert result[0].label == 'car'
