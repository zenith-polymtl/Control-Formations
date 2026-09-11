# Makefile de la formation B3. Même base que celui de B2 (mêmes variables, mêmes
# cibles, mêmes idiomes), avec deux ajouts pour travailler avec ROS 2 : les
# workspaces sont sourcés automatiquement, et une cible de nettoyage. Se roule
# depuis le dossier B3.
#
#   make build          bâtit l'image (obligatoire une première fois en B3)
#   make shell          démarre le conteneur de travail s'il ne roule pas, puis un shell dedans
#   make mavros         démarre mavros dans son propre conteneur (B3.3 et suivants)
#   make mavros-logs    suit les logs de mavros
#   make listen-status  mavros + affichage de /mavros/state
#   make clean          efface build/, install/ et log/ du workspace
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
# donc rien n'est sourcé automatiquement. Ajouts B3, dans l'ordre : le workspace
# b3_tools_ws (bâti dans l'image), puis example_ws s'il a déjà été bâti avec
# colcon. Le vrai dépôt fait pareil (source install/setup.bash dans sa cible shell).
TOOLS_SETUP := /b3_tools_ws/install/setup.bash
WS_SETUP    := /example_ws/install/setup.bash
ROS_SETUP   := source /opt/ros/humble/setup.bash; source $(TOOLS_SETUP); if [ -f $(WS_SETUP) ]; then source $(WS_SETUP); fi

# ===== Cycle de vie =====
build: ## Bâtit l'image
	$(COMPOSE) build

# docker compose up -d ne fait rien si le conteneur roule déjà : la cible est
# répétable, et toutes les cibles qui en dépendent aussi.
up: ## Démarre le conteneur de travail en arrière-plan
	@mkdir -p example_ws
	$(COMPOSE) up -d $(C)

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
shell: up ## Ouvre un shell dans le conteneur, ROS 2 et les workspaces sourcés (autant de fois qu'on veut)
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); exec bash -i'

listen-status: up mavros ## Affiche /mavros/state jusqu'à Ctrl-C
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 topic echo /mavros/state'

# Roule sur l'hôte : les fichiers bâtis appartiennent à votre utilisateur (même
# UID que dev dans le conteneur), pas besoin de sudo. Les sources de src/ ne sont
# pas touchées.
clean: ## Efface build/, install/ et log/ du workspace
	rm -rf example_ws/build example_ws/install example_ws/log

# Cibles qui ne produisent aucun fichier portant leur nom.
.PHONY: build up mavros mavros-logs down sitl shell listen-status clean
