"""
Tests pour trajectory_generator.py — 0 import ROS.
15 tests couvrant les patterns lawnmower et circulaire, plus les utilitaires.
"""
import math

import pytest

from drone_isr.trajectory_generator import (
    Waypoint,
    ZoneConfig,
    compute_strip_width,
    compute_total_distance,
    estimate_mission_duration,
    filter_waypoints_outside_zone,
    generate_circular,
    generate_lawnmower,
)


# ============================================================
# Utilitaires & strip_width
# ============================================================

class TestStripWidth:
    """Tests pour le calcul de la largeur de bande."""

    def test_strip_width_standard(self) -> None:
        # Altitude=10, FOV=60°(1.047rad) -> width ~= 11.547m. Overlap=0.2 -> *0.8 ~= 9.237m
        w = compute_strip_width(10.0, 1.04719, 0.2)
        assert w == pytest.approx(9.2376, abs=0.1)

    def test_strip_width_zero_overlap(self) -> None:
        w = compute_strip_width(10.0, 1.04719, 0.0)
        assert w == pytest.approx(11.547, abs=0.1)


# ============================================================
# Lawnmower
# ============================================================

class TestLawnmowerPattern:
    """Tests pour la génération de la trajectoire en boustrophédon."""

    @pytest.fixture
    def zone_config(self) -> ZoneConfig:
        return ZoneConfig(
            width=30.0,
            height=20.0,
            altitude=8.0,
            strip_width=10.0,
            origin_x=0.0,
            origin_y=0.0,
        )

    def test_lawnmower_covers_full_width(self, zone_config: ZoneConfig) -> None:
        """Vérifie que le pattern couvre toute la largeur de la zone (n_strips correct)."""
        waypoints = generate_lawnmower(zone_config)
        # width=30, strip=10 -> 3 bandes. 2 points par bande -> 6 points.
        assert len(waypoints) == 6
        # Vérifier que les bandes sont espacées de trip_width
        xs = [wp.x for wp in waypoints]
        assert xs[0] == xs[1] == 5.0
        assert xs[2] == xs[3] == 15.0
        assert xs[4] == xs[5] == 25.0

    def test_lawnmower_alternates_direction(self, zone_config: ZoneConfig) -> None:
        """Vérifie que le drone fait des "allers-retours" continus."""
        waypoints = generate_lawnmower(zone_config)
        # Bande 1 : Y de 0 à 20
        assert waypoints[0].y == 0.0
        assert waypoints[1].y == 20.0
        # Bande 2 : Y de 20 à 0
        assert waypoints[2].y == 20.0
        assert waypoints[3].y == 0.0
        # Bande 3 : Y de 0 à 20
        assert waypoints[4].y == 0.0
        assert waypoints[5].y == 20.0

    def test_lawnmower_waypoints_at_correct_altitude(self, zone_config: ZoneConfig) -> None:
        """L'altitude de vol doit être respectée pour tous les waypoints."""
        waypoints = generate_lawnmower(zone_config)
        for wp in waypoints:
            assert wp.z == 8.0

    def test_lawnmower_first_waypoint_is_origin_side(self, zone_config: ZoneConfig) -> None:
        """Le pattern doit commencer depuis origin_y."""
        waypoints = generate_lawnmower(zone_config)
        assert waypoints[0].y == zone_config.origin_y

    def test_lawnmower_invalid_config(self) -> None:
        """Si la zone ou le strip est invalide, retourne liste vide."""
        config = ZoneConfig(width=0, height=10, strip_width=10, altitude=5)
        assert generate_lawnmower(config) == []

    def test_lawnmower_partial_strip(self) -> None:
        """Test avec une largeur qui n'est pas un multiple régulier de strip_width."""
        config = ZoneConfig(width=25.0, height=20.0, strip_width=10.0, altitude=8.0)
        waypoints = generate_lawnmower(config)
        # 25 / 10 = 2.5 -> ceil = 3 bandes.
        assert len(waypoints) == 6
        assert waypoints[-1].x == 25.0 # Le clamp au bord : 5, 15, min(25, 25) = 25


# ============================================================
# Circulaire
# ============================================================

class TestCircularPattern:
    """Tests pour la génération de la trajectoire orbitale."""

    def test_circular_correct_n_points(self) -> None:
        waypoints = generate_circular(0, 0, 10.0, 5.0, n_points=8)
        assert len(waypoints) == 8

    def test_circular_correct_radius(self) -> None:
        """Tous les points doivent être à la distance 'radius' du centre."""
        r = 10.0
        waypoints = generate_circular(5.0, 5.0, r, 5.0, n_points=16)
        for wp in waypoints:
            dist = math.sqrt((wp.x - 5.0)**2 + (wp.y - 5.0)**2)
            assert dist == pytest.approx(r, abs=0.001)

    def test_circular_heading_faces_center(self) -> None:
        """Le cap doit pointer vers l'intérieur du cercle."""
        # Pour le point i=0 avec radius=10, centre=(0,0) -> wp.x=10, wp.y=0. 
        # L'angle du point est 0. Le cap devrait être pi (vers l'origine).
        waypoints = generate_circular(0.0, 0.0, 10.0, 5.0, n_points=4)
        assert waypoints[0].x == 10.0 and waypoints[0].y == 0.0
        assert waypoints[0].heading == pytest.approx(math.pi)

    def test_circular_invalid_params(self) -> None:
        """Tester les paramètres négatifs ou n_points < 3."""
        assert generate_circular(0, 0, radius=0, altitude=5) == []
        assert generate_circular(0, 0, radius=10, altitude=5, n_points=2) == []

    def test_circular_at_correct_altitude(self) -> None:
        waypoints = generate_circular(0, 0, 10, 12.0)
        for wp in waypoints:
            assert wp.z == 12.0


# ============================================================
# Utilitaires de distance & durée
# ============================================================

class TestTrajectoryUtils:
    
    def test_compute_total_distance_triangle(self) -> None:
        """Triangle rectangle (3,4,5)."""
        waypoints = [
            Waypoint(0, 0, 0),
            Waypoint(3, 0, 0),
            Waypoint(3, 4, 0)
        ]
        assert compute_total_distance(waypoints) == 7.0

    def test_compute_total_distance_empty(self) -> None:
        assert compute_total_distance([]) == 0.0
        assert compute_total_distance([Waypoint(1, 1, 1)]) == 0.0

    def test_estimate_duration_consistent_with_distance(self) -> None:
        waypoints = [Waypoint(0, 0, 0), Waypoint(10, 0, 0)]
        # Distance = 10m. Speed = 2m/s -> 5s
        assert estimate_mission_duration(waypoints, 2.0) == 5.0
        assert estimate_mission_duration(waypoints, 0.0) == 0.0

    def test_filter_outside_removes_outliers(self) -> None:
        config = ZoneConfig(width=20, height=20, origin_x=0, origin_y=0, altitude=5, strip_width=5)
        wps = [
            Waypoint(10, 10, 5),   # Inside
            Waypoint(30, 10, 5),   # Outside X
            Waypoint(10, -5, 5)    # Outside Y
        ]
        filtered = filter_waypoints_outside_zone(wps, config)
        assert len(filtered) == 1
        assert filtered[0].x == 10

    def test_filter_outside_keeps_inside(self) -> None:
        config = ZoneConfig(width=20, height=20, origin_x=-10, origin_y=-10, altitude=5, strip_width=5)
        wps = [Waypoint(-5, -5, 5), Waypoint(5, 5, 5)]
        assert len(filter_waypoints_outside_zone(wps, config)) == 2


# ============================================================
# Champ de potentiel (Obstacles)
# ============================================================

from drone_isr.trajectory_generator import Obstacle, compute_potential_field_force, adjust_waypoints_potential_field

class TestPotentialField:
    """Tests pour l'évitement d'obstacles par champ de potentiel."""

    @pytest.fixture
    def single_obstacle(self) -> list[Obstacle]:
        return [Obstacle(x=10.0, y=10.0, z=0.0, radius=2.0, height=10.0)]

    def test_potential_field_repels_near_obstacle(self, single_obstacle: list[Obstacle]) -> None:
        """Le drone doit être repoussé s'il s'approche trop (dans l'influence radius)."""
        # Le drone est à (8.0, 10.0), à l'Ouest de l'obstacle.
        # Le dist_to_center est 2.0. Le rayon de l'obstacle est 2.0. 
        # dist_to_edge = 0. -> Forte répulsion vers l'Ouest (-X).
        fx, fy, fz = compute_potential_field_force(
            position=(8.0, 10.0, 5.0),
            obstacles=single_obstacle,
            repulsion_gain=1.0,
            influence_radius=5.0
        )
        assert fx < -5.0  # Forte force négative en X
        assert fy == pytest.approx(0.0)
        assert fz == 0.0

    def test_potential_field_zero_force_far_obstacle(self, single_obstacle: list[Obstacle]) -> None:
        """Si on est plus loin que l'influence_radius, la force est nulle."""
        fx, fy, fz = compute_potential_field_force(
            position=(0.0, 0.0, 5.0),  # Distance ~14m > (radius 2 + influence 5 = 7)
            obstacles=single_obstacle,
            influence_radius=5.0
        )
        assert fx == 0.0
        assert fy == 0.0

    def test_potential_field_ignores_obstacle_below_drone(self, single_obstacle: list[Obstacle]) -> None:
        """L'obstacle est ignoré si le drone vole nettement au-dessus."""
        # Drone à z=15.0, obstacle_height=10.0 -> ignoré
        fx, fy, fz = compute_potential_field_force(
            position=(10.0, 10.0, 15.0), # Au-dessus exactement
            obstacles=single_obstacle
        )
        assert fx == 0.0
        assert fy == 0.0

    def test_adjust_waypoints_moves_away_from_obstacle(self, single_obstacle: list[Obstacle]) -> None:
        """L'outil d'ajustement statique doit décaler le waypoint du centre."""
        waypoints = [Waypoint(x=9.0, y=10.0, z=5.0)] # Trop près à l'Ouest
        adjusted = adjust_waypoints_potential_field(waypoints, single_obstacle)
        
        # Le waypoint doit avoir été repoussé vers l'Ouest (x diminue)
        assert adjusted[0].x < waypoints[0].x
        assert adjusted[0].y == waypoints[0].y
        assert adjusted[0].z == waypoints[0].z

    def test_adjust_waypoints_preserves_count(self) -> None:
        """La fonction ne doit pas supprimer de waypoints, juste les déplacer."""
        wps = [Waypoint(0, 0, 0), Waypoint(10, 10, 0)]
        obs = [Obstacle(0, 0, 0, 1, 1)]
        res = adjust_waypoints_potential_field(wps, obs)
        assert len(res) == 2

