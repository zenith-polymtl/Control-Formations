# Plan d'apprentissage — Le contrôle de A à Z

*Version du 31 août 2026. Structure du parcours de formation (wiki Notion Zenith) alignée sur les objectifs spécifiques du positionnement initial ING8000B, corrigée suite à l'audit du dépôt `aeac-2026` et de ses 8 submodules (voir `carte-connaissances-aeac-2026.md`). Les ajouts issus de l'audit sont marqués **🔧 Correctif n**.*

## Architecture du parcours

Chaque bloc correspond à une ou plusieurs pages du wiki. Un bloc suppose les précédents acquis, sauf les branches parallèles. Les correctifs sont positionnés à l'endroit exact où ils s'insèrent dans la progression.

```
B0 Prérequis 🔧5
   │
   ▼
B1 Premier contact drone ───────────── Obj 2 (22 sept)
   │
   ▼
B2 Environnement de travail ────────── Obj 3 (17 oct)
   │
   ▼
B3 ROS 2 Humble 🔧1 ────────────────── rattaché à Obj 4
   │
   ├─► B4 Simulation avancée 🔧2 ───── Obj 4 (31 oct)
   │      │
   │      ├─► B5 Architecture mission 🔧3 ── rattaché à Obj 5
   │      │      │
   │      │      ▼
   │      ├─► B6 Réseau & compétition ─ Obj 5 (21 nov)
   │      │      │
   │      │      ▼
   │      └─► B7 Embarqué / Jetson ──── Obj 7 (28 nov)
   │             │
   │             ▼
   │          B8 Vision / IA 🔧6 (spécialisation, hors parcours 101)
   │
   └─► B9 Tuning + analyse post-vol 🔧4 ── Obj 6 (24 oct, parallèle)
```

---

## B0 — Prérequis d'entrée **🔧 Correctif 5**

**Position** : page d'accueil du wiki, lue avant l'introduction de l'objectif 2. Aucun contenu maison : liste de vérification + liens curés vers des ressources externes. Vise à éviter la « pente trop raide » observée dans la formation 1.

- Terminal Linux/bash : navigation, permissions (`chmod`/`chown`), utilisateurs, variables d'environnement.
- Git de base : clone, branch, commit, push (les submodules attendent au B6).
- Python 3 : fonctions, classes, imports.
- Réseau de base : adresse IP, port, `ping`, notion de SSH.

**Autodiagnostic** : 5 questions « sais-tu faire ceci ? » avec lien de rattrapage par question.
**Charge** : ~1 h (curation), absorbée dans l'objectif 1.

## B1 — Premier contact drone — **Objectif 2** (22 sept)

Contenu tel que défini au positionnement : survol ArduPilot/PX4, stations sol (Mission Planner, seule GCS de l'équipe), protocole MAVLink et ses implémentations, survol des simulateurs. Exercice guidé Mission Planner + Zenmav sur SITL, placé avant toute installation complexe (première réussite rapide).

- Préciser dans la page « implémentations MAVLink » la hiérarchie réelle de l'équipe : **Zenmav (débutant/sim) → mavros (mission réelle)**, pymavlink en dessous, MAVSDK anecdotique, dronekit proscrit. Prépare le terrain du 🔧2 sans alourdir l'intro.
- Mesure : taux de complétion à la formation des recrues post-AG (10 sept), comparé aux 40 % de référence.

## B2 — Environnement de travail — **Objectif 3** (17 oct)

Contenu tel que défini : WSL2 + Ubuntu 22.04 (obligatoire — pas 24.04 ; pièges root/user, `/etc/wsl.conf`), Docker (UID/GID, host networking, `chown` du repo), Docker Compose (lecture guidée des 9 fichiers du dépôt), Make et le Makefile (~50 cibles, variables `C=`, `DOMAIN`, `TCP_PORT`, `DRONE_IP`).

- Validation : trois recrues montent leur environnement seules, temps chronométré vs les 5 jours de référence.

## B3 — ROS 2 Humble **🔧 Correctif 1 — le trou principal**

**Position** : nouveau bloc entre l'objectif 3 et l'objectif 4, rattaché administrativement à l'**objectif 4** (prérequis direct de `make mavros-sim`). Format économe en charge : **page-carte de curation**, pas un guide maison — les tutoriels officiels ROS 2 Humble sont excellents ; la valeur ajoutée Zenith est le tri et l'exercice.

- Parcours curé dans les tutoriels officiels : nœuds, topics, services, paramètres, launch, rclpy.
- colcon et structure de package : ament_python vs ament_cmake (`custom_interfaces` comme exemple réel).
- Spécificités maison : workspaces par mission, `link_ws.sh` + `pkgs.txt` (piège du newline final), RMW Cyclone DDS (local) vs rmw_zenoh (inter-machines), `ROS_DOMAIN_ID`.
- TF2 et repères FLU/RDF/ENU, conversions GPS↔local (réutilisés en B4, B8).
- Outils de visualisation : RViz2, Foxglove (`make rviz`, `make foxglove`).
- **Exercice maison** : créer un package rclpy, un node publisher/subscriber, un msg custom dans un clone de `custom_interfaces`, le linker dans un workspace mission.

**Validation** : l'exercice complété seul, dans le conteneur `dev`.
**Charge ajoutée** : ~4 h (curation + exercice), imputée à l'objectif 4 (16 h estimées, marge disponible car les scripts Docker existent déjà en partie).

## B4 — Simulation avancée — **Objectif 4** (31 oct) **🔧 Correctif 2**

Contenu tel que défini : SITL ArduPilot et Gazebo Harmonic, scripts et conteneurs Docker d'installation rapide livrés sur GitHub.

- **Ajout mavros (🔧2)** : section dédiée dans le guide — rôle de mavros comme interface MAVLink réelle en mission, `fcu_url` (TCP 5762 en sim, serial sur Jetson), services `mavros_msgs` (arming, mode, `set_message_interval`), lecture des topics `/mavros/*`. Support concret : `make mavros-sim` existe déjà ; l'exercice consiste à refaire la mission Zenmav de l'objectif 2 via mavros — c'est le **pont Zenmav → mavros** manquant du parcours.
- Validation : installation du simulateur sur machine neuve par un membre qui ne l'a jamais utilisé, sans intervention.

## B5 — Architecture de mission aeac-2026 **🔧 Correctif 3**

**Position** : entre B4 et B6, rattaché à l'**objectif 5** (même public : membre autonome qui se prépare à la compétition ; même réviseurs). C'est le savoir le moins externalisable — celui qui part avec les directeurs en avril. Une page de vue d'ensemble + une sous-page par submodule.

- Vue d'ensemble : chaîne compose → docker → packages → workspaces ; les 4 missions (`dev`, `payload`, `recon`, `water`).
- Rôle et graphe de nœuds de chaque submodule : `custom_interfaces` (contrats msg/srv), `bringup` (launch par mission), `nav_stack` (init/convert/lap), `tools` (heartbeats, DroneHealth), `polar_system`, `TerminationSystem` (geofence, terminaison — lien avec les failsafes du B9).
- Runbook d'opération et de debug (reprend et étoffe `procédure.md`) : `make water`/`gcs-water`, `*-status/-logs/-restart`, diagnostic DDS (`ros2 topic list` muet → réseau/RMW).
- Procédure « ajouter un package » (submodule + pkgs.txt + link).

**Co-rédaction** : sous-pages nav_stack et bringup co-rédigées ou au minimum interviewées avec Haithem Tebib et Nour Karoui — capter leur savoir avant leur départ est l'objet même du projet.
**Charge ajoutée** : ~5 h, imputée à l'objectif 5 (15 h estimées).
**Prérequis de cette page** : corriger d'abord les dérives bloquantes du dépôt relevées à l'audit (launch `UI` cassé, `recon.yml` absent, README obsolète) — sinon on documente un état faux.

## B6 — Réseau & environnement de compétition — **Objectif 5** (21 nov)

Contenu tel que défini : submodules Git en profondeur (`ensure_submodules.sh`, detached HEAD, branches), Zenoh (routeur `rmw_zenohd`, bridges air/ground, configs json5, `ZENOH_CONFIG_OVERRIDE`, endpoints `tcp/7447`), réseau (SIYI HM30, plan d'adressage 192.168.144.x, secours LTE + Tailscale), services systemd (boot, dépendances, retry UART).

- Révision : Haithem Tebib et Nour Karoui.

## B7 — Embarqué / Jetson — **Objectif 7** (28 nov)

Contenu tel que défini : SSH, adressage IP statique local, Tailscale, préparation matérielle.

- Nommer explicitement les spécificités Jetson relevées à l'audit (couverture partielle constatée) : JetPack 5 vs 6 (`ttyTHS0` vs `ttyTHS1`), groupe `dialout`, Docker sur ARM, `ssh zenith@jetson-hexa.local`, service `mavros-jetson` au boot.
- Validation : mise à l'épreuve lors d'une préparation de vol réelle, révisée par un directeur.

## B8 — Vision / IA **🔧 Correctif 6 — hors parcours 101, affiché sur la carte**

**Position** : après B7, marqué « spécialisation — 2e temps » sur la carte Notion. Aucun guide maison cette session (décision C6 du positionnement, périmètre 68 h). La page existe mais ne contient que :

- une phrase de cadrage (« nécessaire pour contribuer au pipeline vision, pas pour être compétitif en contrôle »),
- les pointeurs internes existants : `vision/README.md` (topics ZED utiles, flux de conversion), `Integration_notes.md`, `z2i_pipeline.py`,
- les prérequis pour s'y attaquer (B3 acquis + TF2 solide),
- mention des sujets : ZED SDK / zed-ros2-wrapper (fork `zenith`), YOLO/ultralytics, ONNX/TensorRT.

Ainsi Haithem et Nour valident une carte **complète** (objectif 1) même si le contenu est délégué à plus tard.

## B9 — Tuning & analyse post-vol — **Objectif 6** (24 oct, parallèle) **🔧 Correctif 4**

Contenu tel que défini : PID, filtres, configurations, failsafes, démarche générale de tuning. Prérequis : objectifs 2 à 4.

- **Ajout analyse post-vol (🔧4)** : section « lire ses vols » — tlogs/dataflash dans Mission Planner (le `tuning_cheatsheet.md` actuel fait 3 lignes ; ce contenu le remplace) et rosbags côté ROS 2 (reprendre `rosbags_cheatsheet.md` du dépôt : record, play, introspection). Le tuning sans lecture de logs n'est pas enseignable ; le debug de compétition non plus.
- Lien croisé avec `TerminationSystem` (B5) pour les failsafes logiciels.
- Validation : un membre réalise un tuning encadré en suivant uniquement le guide.
- **Charge ajoutée** : ~2 h, imputée à l'objectif 6 (8 h estimées).

---

## Synthèse des correctifs et impact sur la charge

| 🔧 | Correctif | Position dans l'architecture | Rattachement | Charge |
|---|---|---|---|---|
| 1 | Fondamentaux ROS 2 Humble | Nouveau bloc B3, entre env. de travail et simulation | Obj 4 | +4 h |
| 2 | mavros (pont Zenmav → mission réelle) | Section dans B4 | Obj 4 | +2 h |
| 3 | Architecture des packages de mission | Nouveau bloc B5, avant réseau/compétition | Obj 5 | +5 h |
| 4 | Analyse post-vol (tlogs + rosbags) | Section dans B9 | Obj 6 | +2 h |
| 5 | Prérequis explicites (Linux/git/Python) | Bloc B0, entrée du wiki | Obj 1 | +1 h |
| 6 | Vision/IA affichée comme spécialisation | Bloc B8, page-pointeur seulement | Obj 1 | +0,5 h |

**Charge totale : 68 h → ~82,5 h.** Reste sous les 90 h du plan de cours, mais dépasse la mise en garde « ne t'en mets pas trop ». Si arbitrage nécessaire, l'ordre de sacrifice inverse la valeur : 🔧5/🔧6 (quasi gratuits) et 🔧2 (s'appuie sur l'existant) sont non négociables ; 🔧1 peut se réduire à la curation sans exercice (−2 h) ; 🔧3 peut se limiter à la vue d'ensemble + runbook en reportant les sous-pages par submodule (−3 h) — mais c'est le savoir qui disparaît en avril, donc à ne couper qu'en dernier.

**Jalons inchangés** : les dates des 7 objectifs restent celles du positionnement ; les correctifs s'insèrent dans les guides existants plutôt que de créer de nouveaux livrables, sauf B3 (page-carte) et B5 (pages architecture) qui deviennent des sous-livrables des objectifs 4 et 5. La carte publiée sur Notion (objectif 1, 18 sept) affiche l'architecture ci-dessus, correctifs inclus, pour validation par Haithem Tebib et Nour Karoui.
