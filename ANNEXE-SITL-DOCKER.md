# Annexe : le SITL sans Mission Planner (Docker)

> [!WARNING]
> **Expérimental.** Ce chemin n'a encore été déroulé par personne dans l'équipe. Il est écrit pour les Mac, où Mission Planner n'existe pas, et il servira de premier SITL hors Mission Planner en B4, pour tout le monde. La première personne qui l'essaie est priée de noter ce qui casse dans le fil **Formations et Questions** du salon Contrôle sur Discord, avec les messages exacts : c'est comme ça que l'annexe deviendra fiable.

De B1 à B3, le drone simulé vient de la simulation intégrée de Mission Planner, qui expose MAVLink sur les ports TCP 5762 et 5763 de la machine. Ici, le simulateur ArduPilot (SITL) tourne lui-même dans un conteneur et expose les deux mêmes ports : mavros, Zenmav et le reste des formations ne voient pas la différence. Pour regarder le drone, QGroundControl remplace Mission Planner.

## 1 - Ce qu'il faut

- **Docker Desktop** pour Mac, version 4.34 ou plus récente, avec le réseau hôte activé : Settings → Resources → Network → **Enable host networking**. Sans ce réglage, `network_mode: host` est ignoré et rien ne se voit.
- **QGroundControl** : <https://qgroundcontrol.com/downloads/>. Station sol officielle du projet MAVLink, native sur Mac. L'équipe utilise Mission Planner ; QGC ne sert ici qu'à regarder le drone et à lui donner les ordres de B1 (mode, armement, décollage, aller à un point).
- Le dépôt de la formation cloné, et Docker fonctionnel (`docker run hello-world`).

Sous Docker Desktop, les conteneurs roulent dans une machine virtuelle Linux, et « hôte » veut dire cette machine virtuelle. Les conteneurs de la formation se voient donc entre eux sur `127.0.0.1` sans rien faire de plus. La seule question ouverte est celle de la section 4.

## 2 - Démarrer le SITL

Depuis le dossier de B2 (ou de B3, le fichier est le même) :

```bash
make sitl
docker compose -f compose/sitl.yaml logs -f
```

La première fois, le build clone ArduPilot et compile ArduCopter pour SITL : une dizaine de minutes et plusieurs Go. Ensuite, quelques secondes. Les logs se terminent par les lignes de MAVProxy qui annoncent le drone prêt (`Flight battery 100 percent`, `EKF3 IMU0 is using GPS`). `Ctrl+C` quitte les logs sans arrêter le simulateur.

Le conteneur écoute sur `127.0.0.1:5762` (pour mavros) et `127.0.0.1:5763` (pour Zenmav ou QGC), exactement comme la simulation de Mission Planner. Le drone est posé sur le terrain d'essai CMAC d'ArduPilot, en Australie.

`make down` l'arrête avec le reste.

## 3 - Regarder et piloter avec QGroundControl

Dans QGC : icône Q en haut à gauche → **Application Settings** → **Comm Links** → **Add**. Type **TCP**, adresse `127.0.0.1`, port `5763`, puis **Connect**. Le drone apparaît sur la carte.

Correspondance avec les gestes de B1 dans Mission Planner :

| B1, dans Mission Planner | Dans QGroundControl |
|---|---|
| Menu des modes + **Set Mode** | Le nom du mode en haut de la vue Fly : cliquer, choisir **Guided** |
| **Arm / Disarm** | **Armed / Disarmed** en haut, ou le curseur de décollage ci-dessous qui arme tout seul |
| Clic droit → **Takeoff** | Bouton **Takeoff** à gauche, régler l'altitude, glisser pour confirmer |
| Clic droit → **Fly to here** | Cliquer sur la carte → **Go to location** |
| **RTL** | Bouton **RTL** à gauche |
| **Config → Full Parameter List** | Icône Q → **Vehicle Setup** → **Parameters** |

Le reste de B1 (Zenmav sur `tcp:127.0.0.1:5763`), de B2 (mavros sur `5762`) et de B3 ne change pas : là où le texte dit « regarder dans Mission Planner », regarder dans QGC.

## 4 - Ce qui reste à vérifier

Une seule inconnue technique : que QGroundControl, qui roule sur le Mac, atteigne le port `5763` ouvert par le conteneur en réseau hôte, c'est-à-dire dans la machine virtuelle Docker. La documentation de Docker Desktop dit que les ports d'un conteneur en réseau hôte sont accessibles depuis le Mac ; si ce n'est pas le cas, ajouter au service `sitl` de `compose/sitl.yaml` :

```yaml
    ports:
      - "5763:5763"
```

et retirer la ligne `network_mode: host` de ce seul service. mavros, lui, reste en réseau hôte et voit le SITL par la machine virtuelle.

Deux autres points à confirmer sur une vraie machine : que l'image `ardupilot/ardupilot-dev-base` se bâtit sur Apple Silicon (elle existe en arm64), et le temps de build réel.

## 5 - Pour aller plus loin

- ArduPilot, *SITL Simulator*, <https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html> : les options de `sim_vehicle.py` (lieu de départ `-L`, vitesse `--speedup`, type de véhicule).
- ArduPilot, *Using SITL with Docker*, <https://ardupilot.org/dev/docs/building-setup-linux.html> : le Dockerfile officiel dont `sitl/Dockerfile` s'inspire.
- Le dépôt aeac-2026 fait la même chose avec `make mavros-sim`, sur un SITL lancé à côté : B4 y revient.
