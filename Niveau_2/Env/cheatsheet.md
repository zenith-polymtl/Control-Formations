# Lors du démarrage de VSC :

### Se retrouver dans le bon directory; ex.:
```
cd ~/zenith/Control-Formations/Niveau_2/Env/
``` 
<!-- Changer cela au workspace intéressant, qui risque de changer selon la formation -->

### Démarrer Docker correctement
```
docker compose up
docker exec -it env-zenith-1 bash
```
Note : Le docker exécute par lui-même les commandes de build de ros2
si ce n'est pas déjà créer, exécuter cette commande et modifier le code dans ceci :

>```
>mkdir -p ./ros2_ws/src
>cd ./ros2_ws
>ros2 pkg create --build-type ament_python NOMDUPROGRAMME.py
>```

## Actions possibles après l'exécution du Docker :

- ### Démarrer le programme à partir de ros2
> ```
> ros2 service call /mavros/set_message_interval mavros_msgs/srv MessageInterval "{message_id: 32, message_rate: 20.0}" 
> ```

- ### Exécuter les commandes ros2 pour récupérer les informations
> ```
> ros2 service call /mavros/set_message_interval mavros_msgs/srv MessageInterval "{message_id: 32, message_rate: 20.0}" 
> ```