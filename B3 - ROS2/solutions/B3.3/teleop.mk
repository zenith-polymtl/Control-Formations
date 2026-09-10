# Solution B3.3 : cible teleop. À coller dans le Makefile de B3, dans la section
# « Cibles de travail », et à ajouter à la ligne .PHONY. (Attention : les
# commandes commencent par une tabulation, pas des espaces.)
#
# Tout d'une commande, simulation Mission Planner démarrée :
#   1. up : conteneur + mavros en arrière-plan
#   2. colcon build : le workspace est à jour
#   3. le fichier launch (décollage + contrôle clavier) en arrière-plan, avec ses
#      logs dans /tmp/keyboard_control.log, comme les lancements de fond d'aeac-2026
#   4. la téléop au premier plan, parce qu'elle a besoin du clavier
#   5. nettoyage en sortant (Ctrl+C)
#
# Les logs du launch se lisent depuis un autre terminal :
#   make terminal
#   tail -f /tmp/keyboard_control.log
teleop: up ## Décollage + contrôle clavier, la téléop au premier plan
	$(COMPOSE) exec $(C) bash -lc '$(ROS_SETUP); cd /example_ws && colcon build'
	$(COMPOSE) exec -d $(C) bash -lc '$(ROS_SETUP); ros2 launch b3_bringup keyboard_control.launch.py > /tmp/keyboard_control.log 2>&1'
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 run b3_tools teleop'
	@$(CLEANUP)
