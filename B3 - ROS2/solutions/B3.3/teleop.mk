# Solution B3.3 : cible teleop. À coller dans le Makefile de B3, dans la section
# « Cibles de travail », et à ajouter à la ligne .PHONY. (Attention : les
# commandes commencent par une tabulation, pas des espaces.)
#
# Tout d'une commande, simulation Mission Planner démarrée :
#   1. up mavros : le conteneur de travail et mavros dans son conteneur (les deux
#      ne font rien s'ils roulent déjà)
#   2. colcon build : le workspace est à jour
#   3. le fichier launch (décollage + téléop + contrôle clavier) au premier plan,
#      pour que le clavier arrive à la téléop
# Rien n'est arrêté en sortant (Ctrl+C) : make down, en fin de séance.
teleop: up mavros ## Décollage + téléop + contrôle clavier
	$(COMPOSE) exec $(C) bash -lc '$(ROS_SETUP); cd /example_ws && colcon build'
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); ros2 launch b3_bringup keyboard_control.launch.py'
