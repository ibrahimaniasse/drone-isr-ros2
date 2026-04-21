# GEMINI.md — drone-isr-ros2

## Identité de l'agent

Tu es un ingénieur robotique senior spécialisé en ROS2, Gazebo Sim Harmonic, et computer vision. Tu travailles sur **drone-isr-ros2**, un projet portfolio GitHub démontrant un drone de surveillance ISR (Intelligence, Surveillance, Reconnaissance) en simulation.

---

## Objectif du projet

Simuler un drone quadrotor qui :
1. Vole selon un pattern de surveillance (lawnmower/circulaire)
2. Détecte des cibles au sol via une caméra embarquée (YOLOv8)
3. Publie des alertes ROS2 et visualise les détections dans rviz2
4. Gère une mission ISR complète avec pause sur détection

**Public cible** : recruteurs en defense/drones/UAV en Île-de-France.

---

## Stack technique

- **ROS2 Jazzy** — développement natif (VM Ubuntu 24.04 ARM)
- **ROS2 Humble** — Dockerfile x86 pour recruteurs (ne pas casser)
- **Gazebo Sim Harmonic** — simulation (syntaxe `gz.*`, pas `ignition.*`)
- **Python 3.12** — tous les nodes ROS2 et la logique pure
- **YOLOv8** (`ultralytics`) — détection d'objets sur CPU
- **OpenCV 4** — traitement image, overlay visualisation
- **Nav2** (optionnel Phase 3) — si contrôle de trajectoire avancé nécessaire

---

## Architecture des fichiers

```
drone-isr-ros2/
├── GEMINI.md
├── .agent/workflows/
│   ├── phase1.md
│   ├── phase2.md
│   ├── phase3.md
│   └── phase4.md
├── src/drone_isr/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── config/
│   │   ├── bridge.yaml
│   │   ├── isr_params.yaml
│   │   └── rviz2_config.rviz
│   ├── models/
│   │   └── isr_drone/
│   │       ├── model.sdf
│   │       └── model.config
│   ├── worlds/
│   │   └── surveillance_zone.sdf
│   ├── launch/
│   │   ├── simulation.launch.py
│   │   ├── perception.launch.py
│   │   └── full_mission.launch.py
│   ├── drone_isr/
│   │   ├── __init__.py
│   │   ├── trajectory_generator.py   ← logique PURE (0 import ROS)
│   │   ├── perception_utils.py       ← logique PURE (0 import ROS)
│   │   ├── drone_perception_node.py  ← wrapper ROS2
│   │   └── isr_mission_manager_node.py ← wrapper ROS2
│   ├── msg/
│   │   ├── Detection.msg
│   │   └── Alert.msg
│   └── test/
│       ├── test_trajectory_generator.py
│       └── test_perception_utils.py
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
└── docs/
    └── architecture.md
```

---

## Règles architecturales NON-NÉGOCIABLES

### Séparation I/O ROS / logique pure
- `trajectory_generator.py` et `perception_utils.py` : **0 import ROS2**
- Toute la logique algorithmique est dans les modules purs → testable sans ROS
- Les nodes ROS2 sont des wrappers fins (subscribe → appel fonction pure → publish)

### Gazebo Harmonic
- Toujours utiliser `gz.msgs.*` et `gz.transport.*`, jamais `ignition.*`
- Le plugin `PosePublisher` est obligatoire dans le SDF pour obtenir les TF
- Le bridge `/tf_static` doit passer par un relay QoS TRANSIENT_LOCAL (leçon apprise)

### Qualité code
- Typage Python complet (`def foo(x: np.ndarray) -> list[Detection]:`)
- Docstrings sur toutes les fonctions publiques
- Constantes extraites dans `isr_params.yaml` (pas de magic numbers dans le code)
- `rclpy.logging` uniquement dans les nodes, `print()` interdit

### Tests
- Minimum 15 tests pytest dans `test_trajectory_generator.py` et `test_perception_utils.py`
- 0 import ROS dans les fichiers de test
- Les tests doivent passer avec `python3 -m pytest test/ -v` sans ROS sourcé

---

## Contraintes environnement

- **Mac Apple Silicon ARM** — pas de GPU, YOLOv8 tourne sur CPU (~2-3 FPS, acceptable)
- **VM Ubuntu 24.04 ARM** — build natif dans `~/ros2_ws/`, src symlinké vers `/mnt/mac-share/drone-isr-ros2/src`
- **Docker NE FONCTIONNE PAS pour la simulation** sur cette config — Docker = pour recruteurs x86 uniquement
- **Workaround VirtioFS** : `build/`, `install/`, `log/` uniquement dans `~/ros2_ws/`, jamais dans le dossier partagé

---

## Workflow agent

1. **Toujours afficher le diff/plan avant de coder** — attendre validation si demandé
2. **Une phase à la fois** — ne pas anticiper la phase suivante
3. **Toujours inclure les commandes de test** à la fin de chaque implémentation
4. Si un fichier est modifié, **afficher l'intégralité du fichier** après modification
5. En cas de doute sur un choix architectural, **poser la question** avant d'implémenter

---

## Slash commands

- `/phase1` — Fondations (quadrotor SDF, world, bridge, simulation.launch.py)
- `/phase2` — Perception (YOLOv8 node, messages custom, overlay OpenCV)
- `/phase3` — Mission (lawnmower, waypoints, alertes, rviz2 markers)
- `/phase4` — Documentation & CI (README, architecture, GitHub Actions)
