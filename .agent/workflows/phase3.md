# Phase 3 — Mission ISR

> **Slash command** : `/phase3`
> **Durée estimée** : Jour 3 (5-7h)
> **Dépendances** : Phase 2 validée (/detections actif, FPS stable)

---

## Objectif

Le drone exécute une mission ISR complète de façon autonome : il parcourt sa zone de surveillance selon un pattern lawnmower (passages parallèles couvrant toute la zone), marque les cibles détectées, publie des alertes, et se signale "mission terminée" une fois la zone couverte.

---

## Livrables

1. `drone_isr/trajectory_generator.py` — logique PURE : génère les waypoints lawnmower, circulaires, spiral
2. `drone_isr/isr_mission_manager_node.py` — node ROS2 : ordonnance les waypoints, réagit aux détections, publie alertes
3. `launch/full_mission.launch.py` — lance Gazebo + perception + mission en une commande
4. `config/isr_params.yaml` — mis à jour avec les paramètres mission
5. `test/test_trajectory_generator.py` — 15 tests pytest (0 import ROS)

---

## Critères de succès

```bash
# 1. Tests trajectory_generator
python3 -m pytest test/test_trajectory_generator.py -v
# Résultat attendu : 15/15 PASSED

# 2. Mission complète
ros2 launch drone_isr full_mission.launch.py

# 3. Logs attendus (dans cet ordre)
# [isr_mission_manager] Mission started. Waypoints: 12 (lawnmower pattern)
# [isr_mission_manager] Navigating to waypoint 1/12 (x=5.0, y=0.0, z=8.0)
# [isr_mission_manager] Waypoint 1 reached. Distance error: 0.3m
# [drone_perception] Detected: car (conf=0.78) at world (12.3, -4.1)
# [isr_mission_manager] ALERT: TARGET_DETECTED car at (12.3, -4.1) - hovering 3s
# [isr_mission_manager] Resuming mission. Waypoints remaining: 8
# [isr_mission_manager] MISSION COMPLETE. Targets found: 3. Zone covered: 100%

# 4. Topics attendus
ros2 topic echo /alerts --once
ros2 topic echo /mission_status --once

# 5. Visualisation rviz2
# - Markers rouges sur les cibles détectées (position monde)
# - Trace de la trajectoire parcourue (ligne verte)
# - Waypoints futurs en bleu
```

---

## Spécifications techniques

### trajectory_generator.py (module PUR)

```python
"""
Générateur de trajectoires pour drone ISR — 0 import ROS2.
Entrée : paramètres de zone (taille, altitude, overlap)
Sortie : liste de waypoints (x, y, z)
"""
from dataclasses import dataclass
import numpy as np
from typing import Literal

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    heading: float = 0.0  # cap en radians

@dataclass
class ZoneConfig:
    width: float          # largeur zone en mètres
    height: float         # longueur zone en mètres
    altitude: float       # altitude de vol en mètres
    strip_width: float    # largeur de bande (lié au FOV caméra + overlap)
    overlap: float = 0.2  # overlap entre bandes (20% par défaut)
    origin_x: float = 0.0
    origin_y: float = 0.0

def generate_lawnmower(config: ZoneConfig) -> list[Waypoint]:
    """
    Génère un pattern lawnmower (boustrophédon) couvrant toute la zone.
    Alterne la direction à chaque passage pour minimiser les virages.
    
    Formule strip_width depuis altitude et FOV caméra :
    strip_width = 2 * altitude * tan(fov_h / 2) * (1 - overlap)
    """
    ...

def generate_circular(
    center_x: float,
    center_y: float,
    radius: float,
    altitude: float,
    n_points: int = 16
) -> list[Waypoint]:
    """Génère une trajectoire circulaire autour d'un point d'intérêt."""
    ...

def generate_spiral(
    center_x: float,
    center_y: float,
    max_radius: float,
    altitude: float,
    n_turns: int = 3
) -> list[Waypoint]:
    """Génère une spirale vers l'extérieur depuis le centre."""
    ...

def compute_total_distance(waypoints: list[Waypoint]) -> float:
    """Calcule la distance totale de la trajectoire."""
    ...

def estimate_mission_duration(waypoints: list[Waypoint], speed_ms: float) -> float:
    """Estime la durée de la mission en secondes."""
    ...

def filter_waypoints_outside_zone(
    waypoints: list[Waypoint],
    zone: ZoneConfig
) -> list[Waypoint]:
    """Supprime les waypoints hors de la zone définie."""
    ...
```

### isr_mission_manager_node.py

```python
class ISRMissionManagerNode(Node):
    """
    Gestionnaire de mission ISR.
    
    Subscribe :
      /detections  (drone_isr/DetectionArray) — cibles détectées
      /odom        (nav_msgs/Odometry) — position courante du drone
    
    Publish :
      /cmd_vel     (geometry_msgs/Twist) — commandes de vol
      /alerts      (drone_isr/Alert) — alertes cibles détectées
      /mission_status (std_msgs/String) — état courant de la mission
      /waypoint_markers (visualization_msgs/MarkerArray) — rviz2
      /trajectory_markers (visualization_msgs/MarkerArray) — rviz2
    """

    # États de la state machine de mission
    class State:
        INIT = "INIT"
        NAVIGATING = "NAVIGATING"
        HOVERING_TARGET = "HOVERING_TARGET"
        RECOVERING = "RECOVERING"
        COMPLETE = "COMPLETE"

    def __init__(self):
        super().__init__('isr_mission_manager')

        # Paramètres depuis isr_params.yaml
        self.declare_parameter('zone_width', 40.0)
        self.declare_parameter('zone_height', 30.0)
        self.declare_parameter('flight_altitude', 8.0)
        self.declare_parameter('flight_speed_ms', 2.0)
        self.declare_parameter('waypoint_tolerance_m', 1.0)
        self.declare_parameter('hover_on_detection_s', 3.0)
        self.declare_parameter('camera_fov_h', 1.047)
        self.declare_parameter('strip_overlap', 0.2)

        # Générer les waypoints au démarrage (appel au module pur)
        zone = ZoneConfig(...)
        self._waypoints = generate_lawnmower(zone)
        self._current_wp_idx = 0
        self._state = self.State.INIT
        self._detected_targets: list[tuple[float, float, str]] = []

        # Control loop à 10 Hz
        self._timer = self.create_timer(0.1, self._control_loop)

    def _control_loop(self) -> None:
        """Boucle de contrôle principale — state machine."""
        if self._state == self.State.INIT:
            self._transition_to_navigating()
        elif self._state == self.State.NAVIGATING:
            self._navigate_to_current_waypoint()
        elif self._state == self.State.HOVERING_TARGET:
            self._hover_on_target()
        elif self._state == self.State.COMPLETE:
            self._publish_mission_complete()

    def _navigate_to_current_waypoint(self) -> None:
        """
        Contrôleur P simple vers le waypoint courant.
        Pas de Nav2 — contrôle direct cmd_vel (plus robuste sur VM).
        
        Stratégie :
        1. Monter à l'altitude cible en premier
        2. Puis translater en XY vers le waypoint
        3. Waypoint atteint si distance_3d < tolerance
        """
        wp = self._waypoints[self._current_wp_idx]
        dx = wp.x - self._pose.x
        dy = wp.y - self._pose.y
        dz = wp.z - self._pose.z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        if distance < self._waypoint_tolerance:
            self._on_waypoint_reached()
            return

        # Contrôleur P
        speed = self._flight_speed
        twist = Twist()
        twist.linear.x = np.clip(dx / distance * speed, -speed, speed)
        twist.linear.y = np.clip(dy / distance * speed, -speed, speed)
        twist.linear.z = np.clip(dz * 0.5, -1.0, 1.0)  # altitude plus lente
        self._cmd_vel_pub.publish(twist)

    def _on_detection(self, msg: DetectionArray) -> None:
        """
        Callback sur /detections.
        Si nouvelle cible haute confiance détectée → HOVERING_TARGET.
        """
        for det in msg.detections:
            if det.confidence > 0.6:
                target_key = (round(det.world_x, 1), round(det.world_y, 1))
                if target_key not in self._seen_targets:
                    self._seen_targets.add(target_key)
                    self._publish_alert(det)
                    self._start_hover(det)
                    break

    def _publish_rviz_markers(self) -> None:
        """Publie les markers rviz2 pour waypoints et cibles."""
        ...
```

### isr_params.yaml — paramètres mission

```yaml
isr_mission_manager:
  ros__parameters:
    # Zone de surveillance
    zone_width: 40.0           # mètres
    zone_height: 30.0          # mètres
    zone_origin_x: -20.0       # coin bas-gauche
    zone_origin_y: -15.0

    # Paramètres de vol
    flight_altitude: 8.0       # mètres (compromis vitesse/résolution)
    flight_speed_ms: 2.0       # m/s
    waypoint_tolerance_m: 1.0  # distance pour considérer un waypoint atteint

    # Comportement sur détection
    hover_on_detection_s: 3.0  # secondes de survol sur cible
    min_detection_confidence: 0.60
    dedup_radius_m: 2.0        # distance min entre 2 cibles "différentes"

    # Génération de trajectoire
    camera_fov_h: 1.047        # 60° en radians
    strip_overlap: 0.20        # 20% overlap entre bandes

    # Terminaison
    max_mission_duration_s: 300  # 5 min max (safety)
```

### full_mission.launch.py

Lance dans l'ordre avec les bons délais :
1. Gazebo Sim + world (attendre 5s)
2. ros_gz_bridge (attendre 2s)
3. spawn isr_drone (attendre 3s)
4. robot_state_publisher + tf_static_republisher
5. drone_perception_node (attendre que l'image soit disponible)
6. isr_mission_manager_node
7. rviz2 avec config complète (waypoints + markers + image + TF)

---

## test_trajectory_generator.py — structure (15 tests)

```python
# Lawnmower
def test_lawnmower_covers_full_zone():
    # Vérifie que chaque point de la zone est à max strip_width/2 d'un waypoint
def test_lawnmower_waypoints_at_correct_altitude():
def test_lawnmower_alternates_direction():
    # Les passages pairs vont dans un sens, impairs dans l'autre
def test_lawnmower_no_duplicate_waypoints():
def test_lawnmower_first_waypoint_is_origin():
def test_lawnmower_respects_overlap():

# Circulaire
def test_circular_correct_n_points():
def test_circular_correct_radius():
def test_circular_first_last_not_same():  # pas de fermeture (drone revient au départ)
def test_circular_at_correct_altitude():

# Utilitaires
def test_compute_total_distance_triangle():
def test_compute_total_distance_empty():
def test_estimate_duration_consistent_with_distance():
def test_filter_outside_removes_outliers():
def test_filter_outside_keeps_inside():
```

---

## Visualisation rviz2

Config à inclure dans `rviz2_config.rviz` :

| Display | Topic | Type | Description |
|---------|-------|------|-------------|
| Image | /camera/annotated | Image | Vue caméra avec bboxes |
| MarkerArray | /waypoint_markers | MarkerArray | Waypoints (sphères bleues = futur, vertes = passé) |
| MarkerArray | /trajectory_markers | MarkerArray | Trace du drone (ligne verte) |
| MarkerArray | /target_markers | MarkerArray | Cibles détectées (étoiles rouges) |
| TF | - | TF | Arbre TF complet |
| Axes | - | - | Frame monde |

---

## Fin de phase — checklist avant Phase 4

- [ ] `python3 -m pytest test/test_trajectory_generator.py -v` : 15/15 PASSED
- [ ] Mission démarre automatiquement au lancement
- [ ] Drone suit le pattern lawnmower visible dans rviz2
- [ ] Au moins 1 alerte publiée sur `/alerts` lors d'un survol de cible
- [ ] `mission_status` passe à "COMPLETE" après couverture totale
- [ ] Markers rviz2 : waypoints, trajectoire, cibles détectées
- [ ] Commit : `git commit -m "feat: Phase 3 - lawnmower trajectory, ISR mission manager, alerts, rviz2 markers"`
