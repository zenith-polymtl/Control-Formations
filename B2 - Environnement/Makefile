# Makefile de la formation B2. C'est une version réduite du Makefile d'aeac-2026 :
# mêmes variables, mêmes noms de cibles, mêmes idiomes, mais une poignée de cibles au lieu
# d'une cinquantaine. Se roule depuis le dossier B2.
#
#   make build          bâtit l'image
#   make shell          démarre le conteneur de travail s'il ne roule pas, puis un shell dedans
#   make mavros         démarre mavros dans son propre conteneur, branché sur la simulation
#   make mavros-logs    suit les logs de mavros (la ligne « Got HEARTBEAT » confirme la connexion)
#   make listen-status  mavros + affichage de /mavros/state
#   make down           arrête et supprime les conteneurs

# ===== Variables =====
# ?= n'affecte que si la variable n'est pas déjà définie : les valeurs ci-dessous
# sont donc des défauts, surchargeables sur la ligne de commande.
#   make mavros TCP_PORT=5763
#   make mavros FCU_HOST=172.20.16.1      (WSL sans mode Mirrored)
C        ?= example
TCP_PORT ?= 5762
FCU_HOST ?= 127.0.0.1

# := évalue tout de suite, une seule fois.
COMPOSE_FILE := compose/$(C).yaml
FCU_URL      := tcp://$(FCU_HOST):$(TCP_PORT)
COMPOSE      := docker compose -f $(COMPOSE_FILE)

# Rend ros2 utilisable : docker exec ne passe pas par l'entrypoint de l'image ROS,
# donc rien n'est sourcé automatiquement. Le vrai dépôt fait exactement pareil.
ROS_SETUP := source /opt/ros/humble/setup.bash

# ===== Cycle de vie =====
build: ## Bâtit l'image
	$(COMPOSE) build

# docker compose up -d ne fait rien si le conteneur roule déjà : la cible est
# répétable, et toutes les cibles qui en dépendent aussi.
up: ## Démarre le conteneur de travail en arrière-plan
	@mkdir -p example_ws
	$(COMPOSE) up -d $(C)

# mavros a son propre service dans le fichier Compose, donc son propre conteneur.
# FCU_URL passe par l'environnement : le fichier Compose l'insère dans la commande.
mavros: ## Démarre mavros dans son conteneur, branché sur la simulation
	FCU_URL=$(FCU_URL) $(COMPOSE) up -d mavros
	@echo ">>> mavros roule, branché sur $(FCU_URL). Ses logs : make mavros-logs"

mavros-logs: ## Suit les logs de mavros jusqu'à Ctrl-C
	-$(COMPOSE) logs -f mavros

down: ## Arrête et supprime les conteneurs
	$(COMPOSE) down --remove-orphans

# Mac, expérimental : le SITL ArduPilot dans un conteneur, à la place de la
# simulation de Mission Planner (ANNEXE-SITL-DOCKER.md à la racine du dépôt).
# Fichier Compose à part, même projet : make down l'arrête aussi.
sitl: ## SITL ArduPilot dans un conteneur (Mac, expérimental)
	docker compose -f compose/sitl.yaml up -d

# ===== Cibles de travail =====
# Le - devant une commande dit à make de continuer sans afficher d'erreur quand
# elle sort en erreur, ce qui arrive quand on quitte avec Ctrl-C ou exit.
# Rien n'est arrêté en sortant : make down s'en charge, en fin de séance.
shell: up ## Ouvre un shell dans le conteneur, ROS 2 sourcé (autant de fois qu'on veut)
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); exec bash -i'

listen-status: up mavros ## Affiche /mavros/state jusqu'à Ctrl-C
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 topic echo /mavros/state'

# Cibles qui ne produisent aucun fichier portant leur nom.
.PHONY: build up mavros mavros-logs down sitl shell listen-status
