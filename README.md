# Control-Formations

Le parcours de formation de l'équipe contrôle de Zenith : du premier drone simulé jusqu'à une node ROS 2 qui pilote le drone, dans l'environnement exact du dépôt de compétition. Les documents se lisent directement ici, sur GitHub ; le dépôt se clone en B2.

| Module | Sujet | Durée |
|---|---|---|
| [B0](B0/B0.md) | Prérequis et poste de travail : terminal, Git, Python orienté objet, réseau | 1 h |
| [B1](B1%20-%20Intro/B1.md) | Premier contact : Mission Planner, la simulation, MAVLink, un script Zenmav | 2 h |
| [B2](B2%20-%20Environnement/B2.md) | Environnement de travail : WSL, Docker, Compose, Make, mavros dans un conteneur | 3 h |
| [B3.1](B3%20-%20ROS2/B3.1.md) | ROS 2 : nodes, topics, workspace, publisher et subscriber | 2 h |
| [B3.2](B3%20-%20ROS2/B3.2.md) | ROS 2 : launch, paramètres, services | 1 h 30 |
| [B3.3](B3%20-%20ROS2/B3.3.md) | mavros : topics et services, décollage depuis le terminal, messages à demander | 1 h 30 |
| [B3.4](B3%20-%20ROS2/B3.4.md) | Projet : piloter le drone au clavier avec une node maison | 2 h 30 |

Commencer par [B0](B0/B0.md), qui explique le parcours, où demander de l'aide et quoi poster à la fin de chaque module.

- [DEPANNAGE.md](DEPANNAGE.md) : les problèmes d'environnement (WSL, Docker, ports) qui reviennent dans tous les modules.
- [ANNEXE-SITL-DOCKER.md](ANNEXE-SITL-DOCKER.md) : le simulateur sans Mission Planner (Mac, expérimental).
- [plan_d'apprentissage.md](plan_d'apprentissage.md) : la carte complète du parcours, B0 à B9, et ce qui vient après B3.
