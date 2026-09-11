# Dépannage : l'environnement

Les problèmes qui reviennent dans tous les modules, parce qu'ils touchent l'environnement (WSL, Docker, réseau, ports) et non le contenu. Chaque module a en plus sa propre table « Quand ça casse » en fin de document. Chercher d'abord le message d'erreur exact ; s'il n'est pas ici, le poster tel quel dans le fil **Formations et Questions** du salon Contrôle sur Discord, et il y sera ajouté.

## WSL

| Ce que tu vois | Pourquoi | Quoi faire |
|---|---|---|
| `wsl --install` échoue, ou Ubuntu ne démarre pas après le redémarrage | Fonctionnalités Windows désactivées, ou virtualisation désactivée dans le BIOS | Les deux commandes `dism.exe` de B2 section 1.1, redémarrer ; sinon activer la virtualisation (VT-x ou SVM) dans le BIOS |
| L'invite Ubuntu commence par `root@` | Aucun utilisateur créé au premier démarrage | En créer un (`adduser nom`), le mettre par défaut dans `/etc/wsl.conf` (`[user]` puis `default=nom`), `wsl --shutdown` |
| Tout est lent, ou `colcon build` échoue sur des permissions | Le dépôt est dans `/mnt/c` (le disque Windows) au lieu du home Linux | Cloner dans `~` (B2 section 2.7, étape 3) |
| Un programme Linux ne joint pas la simulation de Mission Planner sur `127.0.0.1` | WSL n'est pas en mode Mirrored (ou Windows 10) | B2 section 1.5 : activer Mirrored, ou utiliser l'adresse de Windows vue de WSL |
| Un changement de réglage WSL ne prend pas effet | WSL n'a pas été redémarré | `wsl --shutdown` dans PowerShell, puis rouvrir |

## Docker

| Ce que tu vois | Pourquoi | Quoi faire |
|---|---|---|
| `permission denied while trying to connect to the Docker daemon socket` | Pas encore dans le groupe `docker` pour cette session | `wsl --shutdown`, rouvrir WSL (B2 section 2.3) |
| `Cannot connect to the Docker daemon` | Le service Docker ne roule pas | `sudo service docker start` |
| Un dossier créé par un conteneur est impossible à supprimer | Il appartient à root | `sudo chown -R $USER:$USER <dossier>` (B2 section 2.5) |
| Le build échoue sur un téléchargement, ou `no space left on device` | Réseau instable, ou disque plein | Relancer le build (le cache reprend) ; `docker system df`, puis `docker system prune` |
| `service "example" is not running` | Le conteneur de travail n'est pas démarré | `make shell` le démarre |
| `ros2: command not found` dans un conteneur | ROS 2 n'est pas sourcé dans ce shell | Entrer par `make shell`, qui source tout ; sinon `source /opt/ros/humble/setup.bash` |
| Les deux formations B2 et B3 se marchent dessus | Mêmes ports pour mavros | `make down` dans l'une avant `make mavros` dans l'autre |

## Réseau et ports

| Ce que tu vois | Pourquoi | Quoi faire |
|---|---|---|
| `connected: false` qui ne change jamais dans `/mavros/state` | mavros ne joint pas la simulation | Simulation démarrée dans MP ? `make mavros-logs` pour voir ce que mavros tente ; mode Mirrored ; `make mavros FCU_HOST=<adresse>` sous Windows 10 |
| `Bind for 0.0.0.0:5762 failed` ou mavros qui redémarre en boucle | Un autre programme occupe déjà le port (un deuxième mavros, un vieux script) | `make down` partout, `docker ps` doit être vide, puis relancer |
| Zenmav ne se connecte pas alors que mavros roule | Les deux veulent le port 5762 | Zenmav sur `tcp:127.0.0.1:5763` (B2 section 2.9) |
| Deux nodes ROS 2 ne se voient pas | `ROS_DOMAIN_ID` différent, ou pas le même réseau | Même valeur dans les deux (le Compose met 2 partout) ; en dehors des formations, voir B6 |

## Quand rien ne marche

Dans l'ordre, en s'arrêtant dès que ça remarche :

1. `make down` dans tous les dossiers de formation, puis `docker ps -a` doit être vide.
2. Fermer Mission Planner, le rouvrir, redémarrer la simulation.
3. `wsl --shutdown` dans PowerShell, rouvrir WSL.
4. Redémarrer Windows.
5. Poster dans le fil Discord : le module, la section, la commande tapée et le message complet.
