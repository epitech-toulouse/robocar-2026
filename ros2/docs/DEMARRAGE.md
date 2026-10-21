# Démarrer le LiDAR et RViz

Ce guide reprend la configuration vérifiée avec la Jetson et le PC. Il faut lancer
les conteneurs sur la Jetson et RViz sur le PC. **Lance RViz depuis un terminal
configuré avec les variables ROS ci-dessous** : un autre terminal ne récupère pas
automatiquement ces variables.

## 1. Démarrer la voiture sur la Jetson

Connecte le LiDAR aux broches UART de la Jetson et démarre la voiture. Puis, sur
la Jetson :

```bash
cd ~/robocar-2026/ros2
docker compose -f docker/compose.yaml --profile control --profile lidar up -d
```

`control` lance notamment `robot_state_publisher` et fournit les transformations
TF du robot. `lidar` démarre le LD19 et publie `/scan`.

Vérifie les conteneurs et les logs :

```bash
docker compose -f docker/compose.yaml ps
docker compose -f docker/compose.yaml logs --tail=30 lidar
```

Les logs du LiDAR doivent indiquer que la communication est normale et que des
scans sont publiés. Vérifie les données directement dans le conteneur :

```bash
docker compose -f docker/compose.yaml exec lidar /entrypoint.sh ros2 topic hz /scan
```

Arrête la commande avec `Ctrl+C` après avoir vu une fréquence proche de 10 Hz.

## 2. Configurer le PC

Le PC et la Jetson doivent utiliser le même `ROS_DOMAIN_ID`. Dans la configuration
actuelle, c’est `31`. Les adresses IP vues pendant la configuration étaient :

- Jetson : `10.15.192.240`
- PC : `10.15.193.160`

Ces adresses peuvent changer. Vérifie-les avec `hostname -I` sur la Jetson et
`ip -4 addr` sur le PC. Mets à jour les pairs ci-dessous si nécessaire.

Dans le fichier `~/robocar-2026/ros2/.env` sur la Jetson, vérifie les lignes :

```dotenv
ROS_DOMAIN_ID=31
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ROS_STATIC_PEERS=10.15.193.160
```

Après toute modification du `.env`, recrée les conteneurs pour charger les
variables :

```bash
docker compose -f docker/compose.yaml --profile control --profile lidar up -d --force-recreate control lidar
```

Sur le PC, dans un terminal **zsh**, configure ROS. Le pair statique doit être
l’adresse IP actuelle de la Jetson :

```zsh
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=31
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=10.15.192.240
ros2 daemon stop
```

Le message `The daemon is not running` est normal si aucun daemon ROS n’était
actif. Vérifie ensuite que le PC découvre la voiture :

```zsh
ros2 node list --spin-time 5
ros2 topic list -t | grep -E 'scan|tf'
ros2 topic hz /scan
```

La liste des nœuds doit notamment contenir `/ldlidar` et
`/robot_state_publisher`. Les topics doivent inclure `/scan`, `/tf` et
`/tf_static`.

## 3. Afficher le LiDAR dans RViz

Lance RViz **depuis le même terminal PC configuré à l’étape 2** :

```zsh
rviz2
```

Dans RViz :

1. Dans **Global Options → Fixed Frame**, choisis `base_link`.
2. Clique sur **Add**, puis ajoute **LaserScan**.
3. Règle son topic sur `/scan`.
4. Si le scan est difficile à voir, augmente **Size (m)**, par exemple à `0.03`.

`base_link` est lié à `laser` par le TF publié par `robot_state_publisher`.
Laisser `map` comme Fixed Frame provoque `Frame [map] does not exist`, car aucun
nœud ne publie actuellement de repère `map`.

## 4. Dépannage rapide

- **Le PC ne voit que `/parameter_events` et `/rosout`** : vérifie que le
  `ROS_DOMAIN_ID` vaut `31` dans le terminal du PC et dans le `.env` Jetson. Vérifie
  aussi que chaque `ROS_STATIC_PEERS` contient l’adresse IP de l’autre machine.
- **La liste contient chaque nœud deux fois** : arrête le cache CLI et reliste :
  `ros2 daemon stop`, puis `ros2 node list --spin-time 5`.
- **RViz affiche `Frame [base_link] does not exist`** : lance RViz depuis le
  terminal configuré, puis vérifie `ros2 topic info /tf_static --verbose`. Il faut
  voir `Publisher count: 1` et `robot_state_publisher`.
- **`/scan` n’apparaît pas ou ne reçoit rien** : sur la Jetson, vérifie les logs
  avec `docker compose -f docker/compose.yaml logs --tail=50 lidar` et teste
  `/scan` dans le conteneur.

## 5. Arrêter

Sur la Jetson :

```bash
docker compose -f docker/compose.yaml --profile control --profile lidar down
```

