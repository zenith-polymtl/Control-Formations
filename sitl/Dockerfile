# syntax=docker/dockerfile:1.6
# SITL ArduPilot dans un conteneur : annexe « SITL sans Mission Planner »
# (ANNEXE-SITL-DOCKER.md). EXPÉRIMENTAL : jamais déroulé par l'équipe.
#
# Image de développement officielle d'ArduPilot (toolchain SITL déjà installée),
# puis clone et compilation d'ArduCopter pour SITL. Le build prend une dizaine
# de minutes et l'image pèse plusieurs Go.
FROM ardupilot/ardupilot-dev-base:latest

# Branche stable d'ArduCopter à compiler. Se change au build :
#   docker compose -f compose/sitl.yaml build --build-arg COPTER_BRANCH=Copter-4.6
ARG COPTER_BRANCH=Copter-4.6

RUN git clone --depth 1 -b ${COPTER_BRANCH} --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
RUN ./waf configure --board sitl && ./waf copter

# Démarre le simulateur avec MAVProxy, qui relaie MAVLink sur deux ports TCP
# écoutés sur toutes les interfaces : 5762 pour mavros, 5763 pour Zenmav ou une
# station sol. Mêmes ports que la simulation de Mission Planner, donc B1 à B3
# ne changent pas. -L CMAC : le terrain de test par défaut d'ArduPilot (Australie).
CMD ["Tools/autotest/sim_vehicle.py", "-v", "ArduCopter", "--no-rebuild", "-L", "CMAC", \
     "--mavproxy-args=--daemon", "--out=tcpin:0.0.0.0:5762", "--out=tcpin:0.0.0.0:5763"]
