# B3 : ROS 2 et mavros

Objectifs, par document :

- **B3.1, premiers pas avec ROS 2** : commandes en ligne de commande (`ros2 node`, `ros2 topic`), workspace colcon, package Python, publisher et subscriber ; le même publisher en C++ en option.
- **B3.2, launch, paramètres et services** : fichier launch dans un package `bringup`, paramètres lus et modifiés en cours d'exécution, service maison dans un package d'interfaces, serveur et client.
- **B3.3, piloter le drone avec mavros** : topics et services qui comptent, QoS, décollage depuis le terminal, demande de messages à l'autopilote (`set_message_interval`).
- **B3.4, projet** : une node qui transforme les touches du clavier en consignes de vitesse (`setpoint_raw/local`), fichier launch, cible Makefile, bonus perte de communication.
