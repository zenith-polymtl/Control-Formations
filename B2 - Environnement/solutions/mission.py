#!/usr/bin/env python3
"""
Solution du défi B2 (étapes 1 et 2) : une mission Zenmav qui roule dans le conteneur.

À copier dans example_ws/mission.py. Le dossier est monté en volume : le fichier
est visible dans le conteneur dès sa création, sans rebuild. Avec la simulation
Mission Planner démarrée :
    make shell
    python3 mission.py

Le port 5763 est le deuxième port TCP du SITL ; le 5762 est pris par mavros
quand il roule (make mavros). Si WSL n'est pas en mode Mirrored, remplacer
127.0.0.1 par l'adresse de Windows vue de WSL (B2 section 2.7, étape 2).

Simulation seulement : sur un vrai drone, c'est le pilote qui arme (B1 section 3.3).
"""
from zenmav.core import Zenmav

drone = Zenmav(ip='tcp:127.0.0.1:5763')

drone.set_mode('GUIDED')
drone.arm()
drone.takeoff(altitude=10)    # bloque jusqu'à 10 m ; changer l'altitude pour l'étape 2 du défi

# Un carré de 40 m de côté, en coordonnées locales NED (nord, est, bas) :
# vers le haut, la troisième coordonnée est négative.
for north, east in [(40, 0), (40, 40), (0, 40), (0, 0)]:
    drone.local_target((north, east, -10))

drone.RTL()                   # attend l'atterrissage et le désarmement
