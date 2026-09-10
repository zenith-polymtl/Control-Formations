# Makefile de la formation B2. C'est une version réduite du Makefile d'aeac-2026 :
# mêmes variables, mêmes noms de cibles, mêmes idiomes, mais trois cibles au lieu
# d'une cinquantaine. Se roule depuis le dossier B2.
#
#   make shell          conteneur + mavros en arrière-plan, puis un shell dedans
#   make listen-status  conteneur + mavros, puis l'écoute de /mavros/state
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
# donc rien n'est sourcé automatiquement. Le vrai dépôt fait exactement pareil.
ROS_SETUP := source /opt/ros/humble/setup.bash

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

listen-status: up ## Affiche /mavros/state jusqu'à Ctrl-C
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 topic echo /mavros/state'
	@$(CLEANUP)

# Cibles qui ne produisent aucun fichier portant leur nom.
.PHONY: build up down shell listen-status
