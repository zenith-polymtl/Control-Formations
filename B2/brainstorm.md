# Brainstorm : B2 - Environnement de travail

Document de travail, point de départ pour l'écriture de B2.md. Basé sur le plan d'apprentissage (version 31 août 2026) et la carte de connaissances aeac-2026. Remplace la version précédente du brainstorm, dont le contenu ROS 2 / MAVROS / défi ballon relève maintenant de B3, B4 et B5.

---

## 1. Positionnement dans le parcours

- **Objectif 3 du positionnement, jalon 17 octobre.**
- En amont : B1 (Mission Planner, MAVLink, Zenmav sur SITL). La recrue a eu sa première réussite rapide, entièrement sous Windows, sans installation complexe.
- En aval : B3 (ROS 2 Humble). Tout B3 se passe dans le conteneur `dev` : B2 doit livrer un poste où `make` et Docker fonctionnent, sinon B3 est bloqué.
- **Niveau visé : autonomie poste de travail.** La recrue monte son environnement seule, du Windows nu jusqu'au conteneur qui roule.
- **Mesure de réussite du bloc** (définie au plan) : trois recrues montent leur environnement seules, temps chronométré, comparé aux 5 jours de référence actuels.

Le contenu est de la *lecture guidée* d'un dépôt existant, pas de la création : on lit les 9 fichiers Compose et le Makefile d'aeac-2026, on n'écrit ni Dockerfile ni Compose.

## 2. Objectifs d'apprentissage (formulation « vous serez capable de »)

1. Installer WSL2 avec Ubuntu 22.04 et s'y connecter depuis VS Code
2. Expliquer image / conteneur / volume et le rôle de Docker dans la stack de l'équipe
3. Installer Docker, régler les permissions (groupe docker, UID/GID) et diagnostiquer les problèmes de propriétaire de fichiers (`chown` du repo)
4. Lire un fichier docker-compose du dépôt et dire quel service il lance, avec quel réseau et quels volumes
5. Utiliser le Makefile : lancer, arrêter, entrer dans un conteneur, consulter les logs, passer les variables (`C=`, `DOMAIN`, `TCP_PORT`, `DRONE_IP`)
6. Monter l'environnement complet de zéro, seul, en moins de X heures (à calibrer, référence actuelle : 5 jours)

## 3. Structure approximative

Le B2.md actuel commence à « 2.2 - WSL », ce qui suppose une numérotation déjà en tête. Proposition compatible :

### Section 1 : Pourquoi cet environnement
- Motivation courte : le drone réel tourne sur Linux (Jetson), le code de mission tourne dans des conteneurs, l'équipe travaille dans un seul dépôt (`aeac-2026`). L'environnement de travail reproduit ça sur le laptop.
- L'argument massue : « ça marche sur mon ordi » n'existe plus quand tout le monde a le même conteneur.
- Annonce du fil rouge : à la fin du bloc, le dépôt est cloné et le conteneur `dev` roule.

### Section 2 : WSL2 + Ubuntu 22.04
- 2.1 : c'est quoi WSL2, pourquoi pas une VM ni un dual boot (recommandation officielle de l'équipe à écrire noir sur blanc).
- 2.2 : installation (déjà amorcée dans B2.md, avec les 3 captures : extension WSL, reconnexion, Close Remote Connection).
- **22.04 obligatoire, pas 24.04** : à répéter, c'est LE mauvais défaut (le store installe « Ubuntu » tout court = 24.04).
- Pièges à documenter : root vs user (première session), mot de passe invisible au clavier (déjà dans le texte), `/etc/wsl.conf` (utilisateur par défaut, systemd), reboot après install.
- Notions de navigation : home Linux vs `/mnt/c`, `\\wsl$` côté Windows, et la règle : **le dépôt vit dans le home WSL**, jamais dans `/mnt/c` (performance et permissions).

### Section 3 : Docker
- Concepts minimum : image, conteneur, volume, réseau. Pas plus que ce qu'il faut pour lire les fichiers du dépôt.
- Installation : trancher Docker Desktop vs docker-ce dans WSL (question ouverte 2).
- Permissions : groupe `docker` + relog, UID/GID dans les conteneurs, symptôme classique des fichiers appartenant à root et le `chown` du repo qui répare.
- Host networking : pourquoi la stack l'utilise (DDS/Zenoh, ports MAVLink) et ce que ça implique sous WSL2.
- Commandes de survie : `docker ps`, `logs`, `exec -it ... bash`, `compose up/down`, et l'hygiène de fin de séance (`down`).

### Section 4 : Docker Compose, lecture guidée du dépôt
- Anatomie d'un service : `build`/`image`, `volumes`, `network_mode`, `environment`, `depends_on`.
- Tour des 9 fichiers de `compose/` : `dev`, `mavros`, `payload`, `water`, `relay`, `zed`, `zed_mini`, `zenoh-air`, `zenoh-ground`. Pour chacun : une phrase sur son rôle, pas une analyse ligne par ligne. Le détail des missions attend B5.
- Faire remarquer la correspondance compose ↔ dockerfiles de `docker/` (6 dockerfiles, x86 GCS vs ARM Jetson) sans creuser les builds.

### Section 5 : Make et le Makefile
- Pourquoi Make : une interface unique devant docker compose, personne ne tape les commandes longues.
- S'appuyer sur la cheatsheet makefile existante du dépôt (la citer, ne pas la dupliquer).
- Les ~50 cibles par familles : build/up/down/shell, launch par mission, services (`*-status`, `*-logs`, `*-restart`), sim (`mavros-sim`, `mavros-gazebo`), visualisation (`rviz`, `foxglove`), ménage (`clean`, `nuke`).
- Variables : `C=`, `DOMAIN`, `TCP_PORT`, `DRONE_IP`. Expliquer concrètement quand on les passe (ex. pointer la sim MP qui roule côté Windows depuis le conteneur).
- On documente l'*usage* des cibles utiles en B2 (`build`, `dev`, shell, logs) ; les cibles mission/sim sont nommées mais renvoyées à B4/B5.

### Section 6 : Montage complet et validation
- Clone du dépôt + `ensure_submodules.sh` donné comme recette (la théorie des submodules attend B6, le dire explicitement).
- Checklist de bout en bout : WSL ok → Docker ok → clone ok → `make build` → shell dans `dev` → une commande témoin qui prouve que tout roule (à choisir : `ros2 --help` dans le conteneur ?).
- **Défi B2** (format défi de B1) : refaire le montage de zéro en se chronométrant, ou aider une autre recrue à monter le sien. La mesure officielle du bloc (3 recrues, chrono vs 5 jours) se fait sur cette checklist.

## 4. Pièges connus à couvrir absolument

- Ubuntu 24.04 installé par défaut au lieu de 22.04
- Mot de passe invisible dans le terminal (déjà dans B2.md)
- Groupe docker : la commande réussit mais rien ne marche avant de fermer et rouvrir la session
- Fichiers du repo appartenant à root après un build : symptôme, cause UID/GID, remède `chown`
- Repo cloné dans `/mnt/c` : lenteur et permissions bizarres
- Espace disque : les images Docker sont grosses, montrer `docker system df` et `make clean`/`nuke`
- Oublier `docker compose down` / laisser des conteneurs orphelins
- Joindre la sim Mission Planner (Windows) depuis le conteneur (WSL) : quelle IP, quel port, lien avec `TCP_PORT`/`DRONE_IP`

## 5. Choix pédagogiques

- Même recette que B1 : objectifs mesurables en tête, définitions inline avec astérisque, captures d'écran à chaque étape cliquable, défis à la fin.
- Beaucoup de captures : c'est un bloc d'installation, chaque écran compte. Convention de nommage déjà en place (`01-extension-wsl.png`, etc.).
- Lecture guidée avant théorie : on ouvre les vrais fichiers du dépôt plutôt que des exemples jouets.
- Encadrés « piège » visuellement distincts pour la liste de la section 4.

## 6. Questions ouvertes à trancher avant l'écriture

1. **Docker Desktop ou docker-ce dans WSL ?** Desktop est plus simple à installer et gère l'intégration WSL ; docker-ce colle mieux au Jetson et évite la question de licence. La recommandation doit être unique et assumée.
2. **Commande témoin de fin de montage** : quelle vérification prouve que l'environnement est bon ? À définir avec le Makefile réel (idéalement une seule cible, genre `make dev` + une commande dans le shell).
3. **Dérives du dépôt** : l'audit note un README obsolète et `compose/recon.yml` manquant. La lecture guidée des 9 fichiers documente l'état réel ; corriger le README avant, ou signaler les écarts dans la formation ? (Même logique que le prérequis noté pour B5.)
4. **Numérotation existante** : le « 2.2 - WSL » actuel colle-t-il à la structure proposée ci-dessus (section 2 = WSL) ? Sinon, renuméroter tôt, avant que les captures se multiplient.
5. **Seuil du chrono** : 5 jours est la référence à battre ; quel objectif afficher aux recrues (une demi-journée ? une soirée ?) sans les décourager.
6. **Windows seulement ?** B2 est écrit pour WSL2. Un membre déjà sous Linux natif ou macOS suit quoi ? Probablement une note d'une ligne (« sautez la section 2, Docker natif ») plutôt qu'un chemin documenté.

## 7. Hors scope B2 (et où ça s'en va)

- ROS 2, colcon, workspaces, `link_ws.sh`/`pkgs.txt` : B3
- mavros, SITL dans la stack, Gazebo : B4
- Rôle des submodules, architecture des missions, runbook : B5
- Théorie Git submodules, Zenoh, réseau : B6
- Écrire des Dockerfiles ou des services systemd : B7 au besoin (Jetson), sinon jamais dans le parcours 101
- L'ancien atelier Niveau_2 (défi ballon) : matière recyclable pour l'exercice de B4 (mission Zenmav refaite via mavros) ou un défi B5, à décider là-bas
