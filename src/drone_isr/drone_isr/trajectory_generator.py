"""
Générateur de trajectoires pour drone ISR — 0 import ROS2.

Entrée : paramètres de zone (taille, altitude, overlap)
Sortie : liste de waypoints (x, y, z)

Patterns supportés :
  - Lawnmower (boustrophédon) — couverture complète de zone
  - Circulaire — orbite autour d'un point d'intérêt
"""
import math
from dataclasses import dataclass


@dataclass
class Waypoint:
    """Point de passage 3D avec cap optionnel."""

    x: float
    y: float
    z: float
    heading: float = 0.0  # cap en radians


@dataclass
class ZoneConfig:
    """Configuration de la zone de surveillance."""

    width: float          # largeur zone en mètres (axe X)
    height: float         # longueur zone en mètres (axe Y)
    altitude: float       # altitude de vol en mètres
    strip_width: float    # largeur de bande effective en mètres
    overlap: float = 0.2  # overlap entre bandes (20% par défaut)
    origin_x: float = 0.0  # coin bas-gauche X
    origin_y: float = 0.0  # coin bas-gauche Y


@dataclass
class Obstacle:
    """Obstacle représenté par un cylindre d'exclusion."""
    x: float
    y: float
    z: float
    radius: float      # rayon de sécurité en mètres
    height: float      # hauteur en mètres


def compute_potential_field_force(
    position: tuple[float, float, float],
    obstacles: list[Obstacle],
    repulsion_gain: float = 5.0,
    influence_radius: float = 5.0,
) -> tuple[float, float, float]:
    """
    Calcule la force répulsive totale exercée par les obstacles sur le drone.
    
    Args:
        position: Position du drone (x, y, z).
        obstacles: Liste des obstacles à éviter.
        repulsion_gain: Force de répulsion (k_rep).
        influence_radius: Distance max à laquelle un obstacle a une influence.
        
    Returns:
        (fx, fy, fz) Vecteur de force.
    """
    fx = 0.0
    fy = 0.0
    fz = 0.0
    px, py, pz = position

    for obs in obstacles:
        # Si le drone est au-dessus de l'obstacle + 1m de sécurité, on l'ignore
        if pz > obs.height + 1.0:
            continue

        dist_to_center = math.hypot(px - obs.x, py - obs.y)
        
        # Distance au bord de l'obstacle de sécurité
        dist_to_edge = dist_to_center - obs.radius
        
        if 0 < dist_to_edge < influence_radius:
            # Force de répulsion (quadratique)
            force_mag = repulsion_gain * (1.0 / dist_to_edge - 1.0 / influence_radius) * (1.0 / (dist_to_edge**2))
            
            # Direction de la force (de l'obstacle vers le drone)
            r_fx = force_mag * (px - obs.x) / dist_to_center
            r_fy = force_mag * (py - obs.y) / dist_to_center
            
            # Vortex tangentiel pour glisser autour de l'obstacle (évite le blocage complet)
            t_fx = -r_fy
            t_fy = r_fx
            
            fx += r_fx + 0.5 * t_fx
            fy += r_fy + 0.5 * t_fy
            
        elif dist_to_edge <= 0:
            # Dans l'obstacle -> répulsion max
            force_mag = repulsion_gain * 100.0
            r_fx = force_mag * (px - obs.x) / max(0.1, dist_to_center) # eviter /0
            r_fy = force_mag * (py - obs.y) / max(0.1, dist_to_center)
            
            # Contournement d'urgence
            t_fx = -r_fy
            t_fy = r_fx
            
            fx += r_fx + 0.5 * t_fx
            fy += r_fy + 0.5 * t_fy

    return (fx, fy, fz)


def adjust_waypoints_potential_field(
    waypoints: list[Waypoint],
    obstacles: list[Obstacle],
    repulsion_gain: float = 2.0,
    influence_radius: float = 4.0,
) -> list[Waypoint]:
    """
    Ajuste statiquement les waypoints pour repousser la trajectoire des obstacles.
    """
    adjusted = []
    
    for wp in waypoints:
        fx, fy, _ = compute_potential_field_force(
            (wp.x, wp.y, wp.z), 
            obstacles, 
            repulsion_gain=repulsion_gain, 
            influence_radius=influence_radius
        )
        
        # On limite le déplacement statique max
        dx = max(-influence_radius, min(fx, influence_radius))
        dy = max(-influence_radius, min(fy, influence_radius))
        
        adjusted.append(Waypoint(
            x=wp.x + dx,
            y=wp.y + dy,
            z=wp.z,
            heading=wp.heading
        ))
        
    return adjusted


def compute_strip_width(altitude: float, camera_fov_h: float, overlap: float) -> float:
    """Calcule la largeur de bande effective depuis l'altitude et le FOV.

    Args:
        altitude: Altitude de vol en mètres.
        camera_fov_h: FOV horizontal de la caméra en radians.
        overlap: Pourcentage de recouvrement entre bandes [0.0, 1.0).

    Returns:
        Largeur de bande effective en mètres.
    """
    ground_width = 2.0 * altitude * math.tan(camera_fov_h / 2.0)
    return ground_width * (1.0 - overlap)


def generate_lawnmower(config: ZoneConfig) -> list[Waypoint]:
    """Génère un pattern lawnmower (boustrophédon) couvrant toute la zone.

    Le drone parcourt la zone en bandes parallèles le long de l'axe Y,
    alternant la direction à chaque passage pour minimiser les virages.

    Args:
        config: Configuration de la zone.

    Returns:
        Liste ordonnée de Waypoint couvrant la zone entière.
    """
    if config.strip_width <= 0 or config.width <= 0 or config.height <= 0:
        return []

    waypoints: list[Waypoint] = []

    # Nombre de bandes nécessaires pour couvrir la largeur (axe X)
    effective_strip = config.strip_width
    n_strips = max(1, math.ceil(config.width / effective_strip))

    for i in range(n_strips):
        # Position X du centre de la bande
        x = config.origin_x + effective_strip / 2.0 + i * effective_strip
        # Clamper au bord de la zone
        x = min(x, config.origin_x + config.width)

        if i % 2 == 0:
            # Passage pair : aller (y croissant)
            y_start = config.origin_y
            y_end = config.origin_y + config.height
        else:
            # Passage impair : retour (y décroissant)
            y_start = config.origin_y + config.height
            y_end = config.origin_y

        waypoints.append(Waypoint(x=x, y=y_start, z=config.altitude))
        waypoints.append(Waypoint(x=x, y=y_end, z=config.altitude))

    return waypoints


def generate_circular(
    center_x: float,
    center_y: float,
    radius: float,
    altitude: float,
    n_points: int = 16,
) -> list[Waypoint]:
    """Génère une trajectoire circulaire autour d'un point d'intérêt.

    Args:
        center_x: Centre X de l'orbite.
        center_y: Centre Y de l'orbite.
        radius: Rayon de l'orbite en mètres.
        altitude: Altitude de vol.
        n_points: Nombre de points sur le cercle.

    Returns:
        Liste de Waypoint formant un cercle (non fermé).
    """
    if n_points < 3 or radius <= 0:
        return []

    waypoints: list[Waypoint] = []

    for i in range(n_points):
        angle = 2.0 * math.pi * i / n_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        # Le heading pointe vers le centre (pour la caméra)
        heading = angle + math.pi
        waypoints.append(Waypoint(x=x, y=y, z=altitude, heading=heading))

    return waypoints


def compute_total_distance(waypoints: list[Waypoint]) -> float:
    """Calcule la distance totale de la trajectoire.

    Args:
        waypoints: Liste ordonnée de waypoints.

    Returns:
        Distance totale en mètres (somme des segments).
    """
    if len(waypoints) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i].x - waypoints[i - 1].x
        dy = waypoints[i].y - waypoints[i - 1].y
        dz = waypoints[i].z - waypoints[i - 1].z
        total += math.sqrt(dx * dx + dy * dy + dz * dz)

    return total


def estimate_mission_duration(waypoints: list[Waypoint], speed_ms: float) -> float:
    """Estime la durée de la mission en secondes.

    Args:
        waypoints: Liste ordonnée de waypoints.
        speed_ms: Vitesse de vol en m/s.

    Returns:
        Durée estimée en secondes. 0.0 si speed <= 0.
    """
    if speed_ms <= 0:
        return 0.0
    return compute_total_distance(waypoints) / speed_ms


def filter_waypoints_outside_zone(
    waypoints: list[Waypoint],
    zone: ZoneConfig,
) -> list[Waypoint]:
    """Supprime les waypoints hors de la zone définie.

    Un waypoint est considéré dans la zone si :
      origin_x <= x <= origin_x + width
      origin_y <= y <= origin_y + height

    Args:
        waypoints: Liste de waypoints à filtrer.
        zone: Configuration de la zone.

    Returns:
        Liste filtrée de waypoints dans la zone.
    """
    x_min = zone.origin_x
    x_max = zone.origin_x + zone.width
    y_min = zone.origin_y
    y_max = zone.origin_y + zone.height

    return [
        wp for wp in waypoints
        if x_min <= wp.x <= x_max and y_min <= wp.y <= y_max
    ]
