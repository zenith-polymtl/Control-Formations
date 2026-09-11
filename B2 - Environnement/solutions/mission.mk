# Solution du défi B2 (étape 3) : cible mission. À coller dans le Makefile de B2,
# dans la section « Cibles de travail », et à ajouter à la ligne .PHONY.
# (Attention : la commande commence par une tabulation, pas des espaces.)
#
# Dépend de up seulement : la mission parle au SITL directement (port 5763),
# mavros n'est pas nécessaire. Rien n'est arrêté en sortant : make down.
mission: up ## Roule example_ws/mission.py dans le conteneur
	-$(COMPOSE) exec -it $(C) bash -lc 'cd /example_ws && python3 mission.py'
