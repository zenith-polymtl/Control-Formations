Je suis parti du principe que je ne connaissais pas ce qui a dans le ballon_pub.py

Méthode choisi : alpha beta filter
On ajoute deux paramètres supplémentaire dans le launch:

    0 < α ≤ 1 
    0 < β ≤ 2 

on joue sur ces gains expérimentales pour avoir la meilleure combinaison. 
En augmentant alpha et beta on augmente la réponse transitoire pour suivre les changements, 
tandis que si tu diminues l'alpha et le beta tu diminues le bruits.
Si beta>1 tu vas amplifier les bruits.  

look ahead  alpha   beta    score
    1       0.2     0.3     732
    2       0.2     0.6     624
    2       0.2     0.4     621
    3       0.2     0.2     3777

Pour l'implémenter : 
ros2 launch bringup control.launch.py
    

    
