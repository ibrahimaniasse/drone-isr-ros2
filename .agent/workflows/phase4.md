# Phase 4 — Documentation & CI

> **Slash command** : `/phase4`
> **Durée estimée** : Jour 4-5 (6-8h)
> **Dépendances** : Phase 3 validée + screenshots/GIF de la démo enregistrés

---

## Objectif

Transformer un repo technique fonctionnel en un projet GitHub "recruiter-ready". Un recruteur doit comprendre le projet en 30 secondes, un ingénieur senior doit être impressionné par la rigueur en 5 minutes.

---

## Livrables

1. README.md — portfolio-grade avec démo visuelle et badges
2. docs/architecture.md — diagramme Mermaid des nœuds et topics
3. docs/design_decisions.md — 4 ADR (Architecture Decision Records)
4. .github/workflows/ci.yml — CI GitHub Actions (lint + pytest + docker build)
5. docs/demo/ — screenshots Gazebo, rviz2, terminal logs, GIF animé

---

## README.md — structure attendue

Rédiger les sections dans cet ordre exact :

**Header** : titre `drone-isr-ros2` + tagline "Autonomous ISR drone simulation — ROS2 Jazzy + Gazebo Harmonic + YOLOv8" + 5 badges (CI, Tests, ROS2 Jazzy, Python 3.12, License MIT).

**Section Demo** : GIF animé 800px + légende "Left: Gazebo Sim | Center: rviz2 | Right: Camera feed (annotated)".

**Section What it does** : 2 phrases maximum. Modèle : "A simulated ISR drone autonomously surveys a 40x30m zone using a lawnmower coverage pattern, detects ground targets (vehicles, persons) with YOLOv8, and publishes georeferenced alerts."

**Section Architecture** : inclure le diagramme Mermaid depuis docs/architecture.md.

**Section Technical highlights** — 4 points différenciants :
- Pure-function design : trajectory_generator.py et perception_utils.py ont zéro import ROS2, testables sans environment ROS
- YOLOv8 on CPU : skip-frame strategy + yolov8n, optimisé pour ARM/x86 sans GPU
- TF static QoS bridge : relay TRANSIENT_LOCAL custom, solution au bug Gazebo Harmonic
- State machine mission : états INIT, NAVIGATING, HOVERING_TARGET, COMPLETE

**Section Quick start** : deux variantes. Native ROS2 (clone + symlink + colcon build + ros2 launch). Docker x86 uniquement (docker compose up).

**Section Results** : tableau 5 lignes. Zone coverage 100% (40x30m). Detection accuracy ~72% mAP YOLOv8n CPU. Mission time ~4 min à 2 m/s 8m altitude. Unit tests 25/25 passing. Processing rate ~3 FPS CPU ARM.

**Section Roadmap** : Phase 5 PX4 SITL, multi-drone coordination, GeoJSON georeferencing output.

**Section About** : 2 phrases contexte portfolio defense/UAV/simulation IDF.

---

## docs/architecture.md — diagramme Mermaid

Le diagramme doit représenter 5 groupes de nodes interconnectés par leurs topics.

**Groupe Gazebo Sim Harmonic** : contient World surveillance_zone, isr_drone model, Camera 10Hz, IMU 100Hz.

**Groupe ROS-Gz Bridge** : contient ros_gz_bridge et tf_static_republisher.

**Groupe Perception Pipeline** : contient drone_perception_node et YOLOv8n CPU.

**Groupe Mission Control** : contient isr_mission_manager_node et trajectory_generator.py.

**Groupe Outputs** : contient rviz2, /alerts, /mission_status.

Flux de données à représenter :
- Camera -> ros_gz_bridge -> /camera/image_raw -> drone_perception_node
- drone_perception_node -> /detections -> isr_mission_manager_node
- drone_perception_node -> /camera/annotated -> rviz2
- isr_mission_manager_node -> /cmd_vel -> ros_gz_bridge -> isr_drone
- ros_gz_bridge -> /tf_static_bridge VOLATILE -> tf_static_republisher -> /tf_static TRANSIENT_LOCAL -> rviz2
- isr_mission_manager_node -> /alerts, /mission_status, /waypoint_markers, /target_markers -> rviz2

### Tableau des topics

| Topic | Type | Hz | Producer | Consumer |
|-------|------|----|----------|----------|
| /camera/image_raw | sensor_msgs/Image | 10 | Gazebo | drone_perception_node |
| /camera/annotated | sensor_msgs/Image | 3-5 | drone_perception_node | rviz2 |
| /detections | drone_isr/DetectionArray | 3-5 | drone_perception_node | isr_mission_manager |
| /alerts | drone_isr/Alert | event | isr_mission_manager | log/external |
| /cmd_vel | geometry_msgs/Twist | 10 | isr_mission_manager | Gazebo |
| /odom | nav_msgs/Odometry | 50 | Gazebo | isr_mission_manager |
| /tf_static | tf2_msgs/TFMessage | latched | tf_static_republisher | tf2 + rviz2 |
| /waypoint_markers | visualization_msgs/MarkerArray | 1 | isr_mission_manager | rviz2 |
| /target_markers | visualization_msgs/MarkerArray | event | isr_mission_manager | rviz2 |

---

## docs/design_decisions.md — 4 ADR

### ADR-001 : YOLOv8n sur CPU sans GPU

**Contexte** : VM ARM Ubuntu 24.04 sans GPU disponible.

**Décision** : yolov8n (nano, 6M params) avec skip-frame 1 image sur 3, device cpu explicite.

**Alternatives** : YOLOv8s/m/l trop lents sous 1 FPS. MobileNet SSD possible mais moins expressif en portfolio. HOG+SVM documenté comme fallback.

**Conséquences** : 3-5 FPS acceptables pour la démo. Sur hardware réel Jetson ou x86 GPU, migrer vers yolov8s avec CUDA.

---

### ADR-002 : Contrôleur P direct au lieu de Nav2

**Contexte** : Nav2 ajoute une complexité de configuration non nécessaire pour un drone en zone ouverte sans obstacles dynamiques.

**Décision** : P-controller direct publié en cmd_vel dans isr_mission_manager_node.py.

**Alternatives** : Nav2 action client (overhead trop important), PX4/ArduPilot SITL (hors scope sur ARM en 1 semaine).

**Conséquences** : Pas de replanning dynamique, acceptable pour zone ouverte. Extension Phase 5 documentée.

---

### ADR-003 : TF static republisher QoS TRANSIENT_LOCAL

**Contexte** : Gazebo Harmonic ne publie pas les TF internes automatiquement. Le bridge ros_gz_bridge publie /tf_static en VOLATILE. tf2_ros exige TRANSIENT_LOCAL.

**Décision** : Node Python tf_static_republisher qui subscribe /tf_static_bridge (VOLATILE) et republish /tf_static (TRANSIENT_LOCAL).

**Alternatives** : Patch bridge.yaml QoS non supporté dans la version Jazzy. URDF + robot_state_publisher seul incomplet pour les sensors SDF.

**Conséquences** : TF tree complet et stable. Pattern réutilisable pour tout projet ROS2 + Gazebo Harmonic.

---

### ADR-004 : Séparation logique pure / nodes ROS2

**Contexte** : Les tests ROS2 requièrent un environment complet. Le CI GitHub Actions ne peut pas lancer Gazebo.

**Décision** : Toute la logique algorithmique dans des modules Python sans import ROS2 (trajectory_generator.py, perception_utils.py). Les nodes sont des wrappers fins.

**Conséquences** : Tests pytest lancables sans ROS, dans le CI standard. Logique réutilisable hors ROS.

---

## .github/workflows/ci.yml — structure attendue

Le fichier CI doit définir 3 jobs qui se lancent sur push vers main et develop, et sur pull_request vers main.

**Job lint** : runner ubuntu-latest, setup Python 3.12, installer flake8 + black + isort, lancer les 3 linters sur src/drone_isr/drone_isr/ avec max-line-length 100.

**Job unit-tests** : runner ubuntu-latest, setup Python 3.12, installer pytest + numpy + opencv-python-headless (sans ultralytics — les tests utilisent des mocks), lancer pytest src/drone_isr/test/ avec --tb=short.

**Job build-docker** : runner ubuntu-latest, docker build avec -f docker/Dockerfile et tag drone-isr-ros2:test.

Note : le build ROS2 natif colcon n'est pas dans le CI standard car il nécessite ROS2 Jazzy. Utiliser ros-tooling/setup-ros@v0.7 si ce workflow est ajouté plus tard.

---

## Checklist GIF démo

Enregistrer avec Kazam ou SimpleScreenRecorder dans la VM. Durée totale : 45 secondes.

Séquence :
1. Vue Gazebo de dessus 10s — drone qui décolle et commence le lawnmower
2. Vue rviz2 15s — waypoints bleus passent en verts, trajectoire verte s'allonge
3. Vue rqt_image_view 10s — caméra annotée avec bbox "car 0.78" lors du survol d'une cible
4. Vue terminal 5s — log "ALERT: TARGET_DETECTED car at (12.3, -4.1)"
5. Vue rviz2 5s — marker étoile rouge sur la cible détectée

Convertir : ffmpeg -i demo.mp4 -vf "fps=15,scale=800:-1" -loop 0 demo.gif

---

## Post LinkedIn — structure

1. Accroche : ce que fait le robot en une phrase
2. Stack : ROS2, Gazebo Harmonic, YOLOv8, ISR, autonomous mission
3. Challenge technique : le plus non-trivial (ADR-003 tf_static QoS)
4. Lien GitHub
5. Hashtags : #ROS2 #Robotique #Drones #UAV #ComputerVision #IDF

---

## Fin de phase — checklist finale

- [ ] README rendu proprement sur GitHub (badges verts, GIF visible, tableau)
- [ ] CI passe sur GitHub Actions (lint + tests verts)
- [ ] Diagramme Mermaid rendu dans docs/architecture.md
- [ ] 4 ADR rédigés dans docs/design_decisions.md
- [ ] GIF démo intégré dans le README
- [ ] Repo public sur GitHub
- [ ] Tag de release : git tag v1.0.0 && git push --tags
- [ ] CV + LinkedIn mis à jour avec le projet
- [ ] Commit final : git commit -m "docs: Phase 4 - portfolio README, architecture, ADRs, CI"
