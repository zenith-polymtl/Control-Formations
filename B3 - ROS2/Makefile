# Makefile de la formation B3. Même base que celui de B2 (mêmes variables, mêmes
# cibles, mêmes idiomes), avec trois ajouts pour travailler avec ROS 2 : le
# workspace est sourcé automatiquement, une cible pour ouvrir des terminaux de plus
# et une cible de nettoyage. Se roule depuis le dossier B3.
#
#   make build          bâtit l'image (obligatoire une première fois en B3)
#   make shell          conteneur + mavros en arrière-plan, puis un shell dedans
#   make terminal       un terminal de plus dans le conteneur déjà démarré
#   make listen-status  conteneur + mavros, puis l'écoute de /mavros/state
#   make clean          efface build/, install/ et log/ du workspace
#   make down           arrête et supprime le conteneur

# ===== Variables =====
# ?= n'affecte que si la variable n'est pas déjà définie : les valeurs ci-dessous
# sont donc des défauts, surchargeables sur la ligne de commande.
#   make listen-status TCP_PORT=5763
C        ?= example
TCP_PORT ?= 5762
FCU_HOST ?= 127.0.0.1

# := évalue tout de suite, une seule fois.
COMPOSE_FILE := compose/$(C).yaml
FCU_URL      := tcp://$(FCU_HOST):$(TCP_PORT)
COMPOSE      := docker compose -f $(COMPOSE_FILE)

# Rend ros2 utilisable : docker exec ne passe pas par l'entrypoint de l'image ROS,
# donc rien n'est sourcé automatiquement. Ajout B3 : si le workspace a déjà été
# bâti avec colcon, ses packages sont sourcés aussi. Le vrai dépôt fait pareil
# (source install/setup.bash dans sa cible shell).
WS_SETUP  := /example_ws/install/setup.bash
ROS_SETUP := source /opt/ros/humble/setup.bash; if [ -f $(WS_SETUP) ]; then source $(WS_SETUP); fi

# Nettoyage partagé par les cibles interactives.
CLEANUP := echo ">>> Nettoyage : arrêt du conteneur"; $(COMPOSE) down --remove-orphans

# ===== Cycle de vie =====
build: ## Bâtit l'image
	$(COMPOSE) build

up: ## Démarre le conteneur en arrière-plan et lance mavros dedans
	@mkdir -p example_ws
	$(COMPOSE) up -d
	$(COMPOSE) exec -d $(C) bash -lc '$(ROS_SETUP); ros2 launch mavros apm.launch fcu_url:=$(FCU_URL) fcu_protocol:=v2.0'
	@echo ">>> mavros roule dans le conteneur, branché sur $(FCU_URL)"

down: ## Arrête et supprime le conteneur
	@$(CLEANUP)

# ===== Cibles de travail =====
# Le - devant une commande dit à make de continuer même si elle sort en erreur,
# ce qui arrive quand on quitte avec Ctrl-C. La ligne de nettoyage roule quand
# même. Si make s'arrête avant, `make down` reste le filet de sécurité.
shell: up ## Ouvre un shell dans le conteneur, mavros déjà lancé
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); exec bash -i'
	@$(CLEANUP)

# Ajout B3. Pas de dépendance à up (sinon un deuxième mavros serait lancé) et pas
# de nettoyage en sortant (sinon fermer ce terminal couperait tous les autres).
# Le conteneur doit donc déjà rouler : make shell d'abord, dans un autre terminal.
# Le - évite que make affiche une erreur en sortant après un Ctrl-C.
terminal: ## Ouvre un terminal de plus dans le conteneur démarré par make shell
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); exec bash -i'

listen-status: up ## Affiche /mavros/state jusqu'à Ctrl-C
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 topic echo /mavros/state'
	@$(CLEANUP)

# Ajout B3. Roule sur l'hôte : les fichiers bâtis appartiennent à votre utilisateur
# (même UID que dev dans le conteneur), pas besoin de sudo. Les sources de src/
# ne sont pas touchées.
clean: ## Efface build/, install/ et log/ du workspace
	rm -rf example_ws/build example_ws/install example_ws/log

# Cibles qui ne produisent aucun fichier portant leur nom.
.PHONY: build up down shell terminal listen-status clean
