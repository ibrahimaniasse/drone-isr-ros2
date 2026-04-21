# Phase 1 — Fondations

> **Slash command** : `/phase1`
> **Durée estimée** : Jour 1 (4-6h)
> **Dépendances** : aucune

---

## Objectif

Un drone quadrotor spawne dans Gazebo Harmonic, sa caméra et ses topics sont bridgés vers ROS2, et le TF tree est complet dans rviz2. Fin de phase : image caméra visible + drone qui répond à cmd_vel.

---

## Livrables

1. `models/isr_drone/model.sdf` — modèle quadrotor (corps + 4 bras visuels + caméra nadir + IMU + PosePublisher)
2. `worlds/surveillance_zone.sdf` — terrain 50x50m avec 6 cibles statiques (3 vehicules + 3 personnes)
3. `config/bridge.yaml` — bridge ROS/Gz (camera, imu, cmd_vel, odom, tf_static)
4. `launch/simulation.launch.py` — Gazebo + bridge + spawn + tf_static_republisher + rviz2
5. `package.xml` + `setup.py` — package ROS2 correctement configure
6. `config/rviz2_config.rviz` — vue image camera + TF tree + Grid

---

## Criteres de succes

Valider dans cet ordre :

```bash
# 1. Build sans erreur
cd ~/ros2_ws
colcon build --symlink-install --packages-select drone_isr
source install/setup.bash

# 2. Simulation demarre
ros2 launch drone_isr simulation.launch.py

# 3. Topics presents
ros2 topic list
# Attendu : /camera/image_raw, /imu, /odom, /tf, /tf_static

# 4. TF tree complet (timeout 10s minimum sur VM ARM)
ros2 run tf2_tools view_frames
# Attendu : odom -> base_link -> camera_link

# 5. Image camera visible (vue du sol depuis 1.5m)
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# 6. Drone repond a cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.3}, angular: {z: 0.0}}" --once
```

---

## Specifications techniques

### Modele SDF quadrotor

Structure du modele (implementer dans cet ordre) :

**Corps principal** `base_link` : boite 0.4x0.4x0.1m, masse 1.5 kg, couleur noir/rouge. Spawn a z=1.5m.

**4 bras + rotors** (visuels uniquement) : `arm_fl`, `arm_fr`, `arm_rl`, `arm_rr` a plus ou moins 0.25m en X et Y. Cylinders fins pour les bras, disques plats pour les rotors.

**Camera nadir** `camera_link` : pose `0 0 -0.06 0 1.5708 0` (rotation 90 degres pour pointer vers le sol). Resolution 640x480, FOV 60 degres (1.047 rad), update rate 10 Hz. Plugin : `gz-sim-camera-system`.

**IMU** `imu_link` : update rate 100 Hz. Plugin : `gz-sim-imu-system`.

**Plugin PosePublisher** (obligatoire pour les TF) :
- `publish_link_pose: true`
- `publish_sensor_pose: true`
- `static_publisher: true`
- `static_update_frequency: 1`
- `update_frequency: 50`

**Plugin de controle** — tester dans cet ordre :
1. `gz-sim-velocity-control-system` (le plus simple)
2. `gz-sim-multicopter-motor-model-system` (si disponible sur Jazzy/Harmonic)
3. Fallback : node Python qui convertit cmd_vel en ApplyLinkWrench

Pour verifier les plugins disponibles :

```bash
gz sim --list-plugins | grep -i "velocity\|multicopter"
```

### World SDF — surveillance_zone

Terrain plat 50x50m avec les cibles statiques suivantes :

```
Vehicule 1 : box 4x2x1.5m, bleu,  position (12, -4, 0.75)
Vehicule 2 : box 4x2x1.5m, rouge, position (-8, 10, 0.75)
Vehicule 3 : box 3x1.5x1m, gris,  position (18, 8, 0.5)
Personne 1 : cylinder r=0.3 h=1.7m, vert, position (5, -12, 0.85)
Personne 2 : cylinder r=0.3 h=1.7m, vert, position (-15, -5, 0.85)
Personne 3 : cylinder r=0.3 h=1.7m, vert, position (10, 15, 0.85)
Eclairage  : directional light, direction (0.5, -0.5, -1), diffuse blanc
```

### config/bridge.yaml

```yaml
- ros_topic_name: /camera/image_raw
  gz_topic_name: /world/surveillance_zone/model/isr_drone/link/camera_link/sensor/camera/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /imu
  gz_topic_name: /world/surveillance_zone/model/isr_drone/link/imu_link/sensor/imu/imu
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS

- ros_topic_name: /cmd_vel
  gz_topic_name: /model/isr_drone/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /odom
  gz_topic_name: /model/isr_drone/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS

# TF bridge — sera relaie en TRANSIENT_LOCAL par tf_static_republisher
- ros_topic_name: /tf_static_bridge
  gz_topic_name: /world/surveillance_zone/dynamic_pose/info
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
```

### launch/simulation.launch.py — ordre de demarrage

Nodes a lancer dans cet ordre avec les delais :

1. `gz sim surveillance_zone.sdf` (attendre 5s)
2. `ros_gz_bridge` avec bridge.yaml (attendre 2s)
3. `gz_spawn_entity` — isr_drone a `(0, 0, 1.5)` (attendre 3s)
4. `robot_state_publisher`
5. `tf_static_republisher` node Python (voir template ci-dessous)
6. `static_transform_publisher` — odom -> base_link initial
7. `rviz2` avec rviz2_config.rviz

### tf_static_republisher.py — template obligatoire

```python
#!/usr/bin/env python3
"""
Relay /tf_static_bridge (VOLATILE) vers /tf_static (TRANSIENT_LOCAL).
Obligatoire : Gazebo Harmonic bridge publie en VOLATILE,
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
            TFMessage, '/tf_static_bridge', self._cb, volatile_qos)
        self.pub = self.create_publisher(TFMessage, '/tf_static', transient_qos)
        self.get_logger().info(
            'Bridging /tf_static_bridge (VOLATILE) -> /tf_static (TRANSIENT_LOCAL)')

    def _cb(self, msg: TFMessage) -> None:
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TfStaticRepublisher())
    rclpy.shutdown()
```

---

## Points de vigilance

**TF static des le depart** — integrer le `tf_static_republisher` au launch file Phase 1. C'est le bug #7 de l'explorer et il coute des heures si oublie.

**Spawn altitude** — spawner a z=1.5m minimum, sinon collision sol au demarrage.

**Camera update_rate** — ne pas depasser 10 Hz dans le SDF. Le renderer software ARM donnera 2-5 FPS reels, c'est normal et acceptable.

**Nettoyage avant relance** :

```bash
pkill -9 -f "gz\|ruby\|ros2"
sleep 3
```

---

## Fin de phase — checklist avant Phase 2

- [ ] `colcon build` sans warning critique
- [ ] Drone visible dans Gazebo a 1.5m d'altitude
- [ ] `/camera/image_raw` affiche une vue du sol
- [ ] `ros2 run tf2_tools view_frames` : `odom -> base_link -> camera_link` present
- [ ] Log `tf_static_republisher` : "Bridging /tf_static_bridge (VOLATILE) -> /tf_static (TRANSIENT_LOCAL)"
- [ ] Drone bouge quand on publie sur `/cmd_vel`
- [ ] Commit : `git commit -m "feat: Phase 1 - quadrotor SDF, surveillance world, bridge, TF tree"`
