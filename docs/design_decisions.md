# Architecture Decision Records (ADR)

## ADR-001 : YOLOv8n sur CPU sans GPU

**Contexte** : Le simulateur est pensé pour tourner nativement sur un cluster ARM (Apple Silicon) ou sur une VM Ubuntu 24.04 classique, tous deux sans passthrough GPU / CUDA.

**Décision** : Utilisation du modèle `yolov8n` (nano, 6M de paramètres) couplé à une stratégie de skip-frame (inférence sur 1 image sur 3) et configuration explicite du `device="cpu"`. 

**Alternatives** : 
- Les modèles YOLOv8s/m/l tournent à <1 FPS sur CPU, causant des latences fatales pour le tagging TF statique de la cible.
- MobileNet SSD (moins expressif, détection de piétons/véhicules de haut moins performante sur l'altitude).
- HOG+SVM (trop archaïque pour un portfolio ROS2 moderne).

**Conséquences** : La vision tourne à un taux acceptable de ~3-5 FPS sans surcharger les threads Gazebo. Sur un déploiement réel (ex: Jetson Orin ou x86 avec GPU discret), la bascule vers `yolov8s` + CUDA est immédiate via le paramétrage de `process_every=1`.

---

## ADR-002 : Contrôleur P direct au lieu de Nav2

**Contexte** : La mission ISR consiste à quadriller une zone ouverte (Lawnmower) sans contrainte spatiale horizontale intra-waypoints, le drone volant à 25m d'altitude au-dessus des obstacles statiques normaux.

**Décision** : Utilisation d'un contrôleur proportionnel (P-controller) calculant la Twist et le Heading dynamiquement dans `isr_mission_manager_node.py` via `_drive_to`.

**Alternatives** : 
- Nav2 action client complet : overhead lourd, génère des costmaps globales 3D complexes non adaptées pour ce use-case simple en espace ouvert.
- PX4/ArduPilot SITL : overkill pour valider la pipeline ROS2 interne.

**Conséquences** : Pas de replanning dynamique massif, mais maintien exact de la consigne (Altitude, Vx, Vy). Facilite la lecture du code pour des use-cases géométriques stricts. Le passage à PX4 SITL est ciblé en Phase 5.

---

## ADR-003 : TF static republisher QoS TRANSIENT_LOCAL

**Contexte** : Gazebo Harmonic ne publie pas les transformations internes statiques automatiquement vers ROS2. Le package natif `ros_gz_bridge` publie les TF internes de Gazebo via le topic bridge en mode QoS `VOLATILE`. Malheureusement `tf2_ros` exige impérativement une politique `TRANSIENT_LOCAL` pour `/tf_static`.

**Décision** : Création d'un mini-node Python `tf_static_republisher` qui *subscribe* statiquement au `/tf_static_bridge` (en mode `VOLATILE`), capture les transforms du drone (wing, tail, rotors, camera, imu, pusher_prop), puis les *republie* immédiatement sur le vrai `/tf_static` configuré officiellement avec QoS `TRANSIENT_LOCAL`.

**Alternatives** : 
- Patcher le code C++ YAML QoS de `ros_gz_bridge` (complexe, requiert une fork).
- Utiliser `robot_state_publisher` seul avec l'URDF (ne synchronise pas avec les liens dynamiques internes Gazebo comme les hélices ou le gimbal SDF).

**Conséquences** : Le TF tree est complet, continu, rviz2 affiche les meshes sans se plaindre de frames manquantes, et le log `/tf_static` maintient sa pérennité. Ce pattern est indispensable et réutilisable pour tous les projets ROS2-Jazzy/Gazebo-Harmonic.

---

## ADR-004 : Séparation logique pure / nodes ROS2

**Contexte** : Un écueil classique de la programmation ROS2 est le couplage serré entre les callbacks ROS et la logique algorithmique mathématique. Cela empêche les tests unitaires via pytest de s'exécuter dans un CI propre sans environnement simulé (ex: GitHub Actions standard ubuntu-latest).

**Décision** : L'architecture est scindée. Toute la logique métier est codée en fonctions Python pures sans le module `rclpy` (ex: `trajectory_generator.py` pour générer le lawnmower, `perception_utils.py` pour le calcul de reprojection). Les fichiers `..._node.py` n'ont pour unique rôle que de brancher (wrapper) ces fonctions pures aux topics/publishers ROS2.

**Conséquences** : Les tests unitaires (fichiers `test_....py`) passent sans dépendre de l'exécution de Gazebo. La logique est portable hors ROS2 si la stack logiciel évolue vers une version non-ROS (ZMQ, MQTT).
