# Atelier ROS 2 + MAVROS (avec ZenMav)

### Suivi d’une cible mobile (“Ballon”) et évaluation automatique

> **But** — Concevoir un nœud ROS 2 qui suit une cible mobile et démontrer la maîtrise des bases ROS 2 dans un contexte drone, en utilisant **MAVROS** (pont ROS 2↔︎MAVLink) et/ou des abstractions haut‑niveau comme **ZenMav**. Le tout est évalué par un nœud moniteur fourni.

## 1) Docker
 **Installation de Docker** : 

 Suivre les instructions using apt

 https://docs.docker.com/engine/install/ubuntu/

Pour build le docker, être dans un terminal au même niveau que le dockerfile :

'''
docker compose up --build
'''

Pour lancer un docker déjà bati
'''

'''
docker compose up
'''

Pour fermer un docker (important après une séance)

'''
docker compose down

'''

Pour ouvrir un terminal dans ce docker, ouvrir autant que nécessaire

'''
docker exec -it env-zenith-1 bash

'''


---

## 1) Ce que vous allez apprendre
- Architecture ROS 2 : **nœuds**, **topics**, **messages**, **QoS**, **callbacks**, **timers**, **paramètres**.
- Intégration drone : notions **MAVLink ↔︎ MAVROS**, frames **ENU/NED**, timestamps et horloge.
- Stratégies de suivi de cible simples (poursuite directe, lissage, limites), sans présumer d’une API unique.
- Lecture d’une **note** de performance émise automatiquement.

---

## 2) Les trois fichiers fournis (aperçu)
- **Générateur de cible (“Ballon”)** : publie une pose cible qui évolue dans le temps et signale le début/fin d’une session.
- **Moniteur** : souscrit aux flux pertinents (cible + drone) et calcule des métriques (erreurs, cumulés, résumé). Sort un rapport (p. ex. CSV).
- **Solution d’exemple** : une implémentation minimale de suivi. **À consulter seulement après votre propre tentative. (Idéalement)**

> Les noms de topics exacts, frames et détails d’implémentation sont visibles directement dans les fichiers.

---

## 3) Rappels ROS 2 essentiels (rclpy)
- **Nœud** : classe qui crée des **publishers**, **subscribers**, **timers** et **services**.
- **Publisher / Subscriber** : `create_publisher(Type, "…", queue)`, `create_subscription(Type, "…", callback, queue)`.
- **Callback** : fonction appelée à la réception d’un message ; évitez les calculs bloquants.
- **Timer** : `create_timer(période_s, callback)` pour une boucle d’asservissement périodique (p. ex. 20–50 Hz).
- **QoS** : choisissez une profondeur raisonnable (KEEP_LAST), préférer **Best Effort** pour flux rapides (IMU) et **Reliable** pour états lents.
- **Paramètres** : exposez des gains/vitesses/altitudes via `declare_parameter/get_parameter` pour itérer sans recompiler.
- **Horodatage** : utilisez les timestamps des messages (et le `dt`) pour la cohérence temporelle.
- **Frames** : ROS 2 côté MAVROS est généralement en **ENU** ; vérifiez vos conversions si votre contrôleur attend **NED**.

---

## 4) Couches drone : MAVROS, MAVLink et ZenMav
- **MAVROS** : pont ROS 2↔︎MAVLink. Il expose la télémétrie (pose, IMU, état) et des interfaces de commande (positions/vitesses/attitude) sous forme de topics/services/actions ROS 2. N'expose pas les messages de télémétrie par défault. Pour avoir la télémétrie faire :
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}" 
- **MAVLink** : protocole bas niveau (messages, modes, armement, consignes).
- **ZenMav** : **option** haut‑niveau (Python) qui encapsule des séquences courantes (mode/armement/consignes). Vous pouvez **tout** faire avec MAVROS seul, **ou** utiliser ZenMav pour simplifier—au choix du participant.

> L’atelier **n’impose pas** d’API de commande. Choisissez **MAVROS pur** ou **ZenMav** selon vos préférences mais surtout les situations. Plusieurs fonctions de Zenmav sont bloquantes, et devront être évitées afin de ne pas bloquer la réception et l'envoie de message d'une node. Dépend de la situation
---

## 5) Architecture de l’atelier (vue logique)
1. Un nœud **Ballon** publie périodiquement une *pose cible* dans une frame fixe (ex. `map`, ENU).
2. Votre nœud **Participant** souscrit cette pose et génère des commandes de guidage (position/vitesse/yaw…) via MAVROS **ou** ZenMav.
3. Un nœud **Moniteur** observe cible et drone, puis calcule une **note** sur une fenêtre temporelle définie.

---

## 6) Tâches à réaliser (dans l’ordre)
1. **Initialisation** : Créer un nœud ROS 2 (rclpy), configurer publishers/subscribers, possible de déclarer des paramètres (altitude cible, gains, limites de vitesse, etc.). Ajouter ce noeud dans le setup.py. Créer son propre launch file avec son noeud. À ce point vous pourrez lancer votre simulation avec : ros2 launch bringup nom_de_file.launch.py

1. **Rendez vous** : Se rendre à la coordonnée (10,20, -50) , donnée en système NED

2. **Annonce d'amorce** : Publier son nom (String), au topic /arrival. Un décompte commencera peu après pour le début du défi.

3. **Suivi de cible** : La position d'un ballon virtuel sera publiée sur le topic /Ballon_pose. Votre objectif est d'être le plus proche de ce point à sa publication.

La trajectoire est :
 - Continue, mais publiée discrète
 - Réalisable
 - Théoriquement parfaite (sans bruit rapide)

Les points sont de type PoseStamped, et seront données en système ENU.

 4. **Être proche du ballon** : Vous serez évaluez sur votre distance à la cible, à chaque publication de la cible, même si vous la connaissez pas d'avance. Vous n'aurez aucune information future sur la trajectoire.

 5. **RTL à la fin** : Après 2 minutes, les points cesseront d'être publiés, sans avertissement. À ce moment, le drone doit retourner au décollage pour y atterir.

 6. **Évaluation de performance** : Un csv de vos performances sera produit dans le répertoir ros2_ws. Le fichier python permet de visualiser la performance. Nécessite d'avoir les modules pandas, numpy et matplotlib. Installation standard avec pip dans le terminal









---

## 7) Critères de réussite (général)
- **Exactitude** : distance moyenne au Ballon faible, dérive limitée.
- **Stabilité** : pas d’oscillations persistantes, commande bornée (vitesses/altitudes limitées), continuité des consignes.
- **Robustesse** : comportement raisonnable si la cible saute/ralentit/accélère ; gestion des messages manquants ponctuels.
- **Qualité ROS 2** : structure claire (timers vs callbacks), QoS adaptés, paramètres configurables, logs utiles.
- **Hygiène** : code lisible, commentaires brefs mais précis, séparation calcul/IO.

> Le **Moniteur** exporte un rapport que vous utiliserez pour interpréter votre performance.

---

## 8) Conseils de conception
- **Découplage** : stockez les dernières mesures dans l’état du nœud et traitez-les dans la boucle timer.
- **Filtrage** : appliquez un lissage simple (p. ex. moyenne exponentielle) sur la cible si nécessaire.
- **Limites** : imposez des plafonds de vitesse/accélération et d’altitude. Prévoyez une zone de sécurité.
- **Temps** : calculez **dt** (à partir des horodatages) pour des lois dépendant de la vitesse.
- **Essais progressifs** : commencez par poursuite naïve, puis ajoutez lissage/anticipation au besoin.

---

## 9) Tests & validation (général)
- Vérifiez que la fréquence de votre **timer** est stable (journalisez la période effective).
- Inspectez les frames et les champs `header.stamp` des messages.
- Surveillez l’usage CPU et la latence de bout‑en‑bout (éviter les surcharges dans les callbacks).
- Interprétez les métriques du **Moniteur** pour guider vos ajustements.

---

## 10) Sécurité & bonnes pratiques
- **Simulation d’abord** : validez en simulateur avant tout essai matériel.
- **Limiteurs** : vitesse/altitude bornées, conditions d’armement/mode gérées proprement.
- **Arrêt sûr** : prévoyez un état “stop/safe” et des garde‑fous si la cible devient invalide.

---

## 11) Pour aller plus loin (optionnel)
- Passage position→vitesse→accélération selon la disponibilité des interfaces MAVROS.
- Petites anticipations (v≈constante) et saturation douce.
- Paramétrage dynamique (reconfiguration) et traces supplémentaires dans le rapport.

---

### Remarque finale
- Le détail exact des topics, frames et signaux d’orchestration est dans les **fichiers fournis** (Ballon, Moniteur).
- La **solution** est un *exemple* ; privilégiez votre propre conception avant de la consulter.

