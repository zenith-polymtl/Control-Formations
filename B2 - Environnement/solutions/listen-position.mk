# Solution du défi B2 (bonus) : cible listen-position. À coller dans le Makefile
# de B2 et à ajouter à la ligne .PHONY. (Tabulation devant les commandes.)
#
# Dépend de up et de mavros : ni l'un ni l'autre ne fait quoi que ce soit quand
# son conteneur roule déjà, donc la cible se lance pendant que make mission
# tourne dans un autre terminal.
#
# ArduPilot n'envoie la position que si on la demande : la requête du message 32
# (LOCAL_POSITION_NED, ici à 10 Hz) précède l'écoute. Le « ; » enchaîne les deux
# commandes dans le même shell, et le « \ » continue la ligne.
listen-position: up mavros ## Demande la position locale à 10 Hz et l'affiche
	-$(COMPOSE) exec -it $(C) bash -lc '$(ROS_SETUP); \
	  ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 10.0}"; \
	  ros2 topic echo /mavros/local_position/pose'
