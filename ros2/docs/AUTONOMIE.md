# Suivi des flèches et sécurité LiDAR

Le profil `autonomy` suit les flèches de ruban blanc depuis `/oak/rgb/image_raw`.
Il utilise la partie basse de l'image et publie un masque de détection sur
`/autonomy/arrow_mask` ainsi qu'un état lisible sur `/autonomy/status`.

Le contrôleur publie `/drive` à 20 Hz. Il ne démarre pas sans image récente,
scan LiDAR récent et flèche détectée récemment. Le bouton **RB** doit rester
maintenu pour déverrouiller `/drive` dans le multiplexeur.

La brique `control` place `lidar_safety_gate` après le multiplexeur et avant le
VESC. Elle met la vitesse à zéro si un obstacle est détecté dans le secteur
avant, si le scan est périmé ou si la commande amont se tait. Elle s'applique
donc aussi à la conduite manuelle. Les distances de départ sont 0,70 m pour
l'arrêt, 0,95 m pour le redémarrage après arrêt et 1,50 m pour commencer à
ralentir ; elles se règlent dans `config/safety.yaml`.

Le LiDAR 2D réagit à tout retour dans son plan de balayage : il ne reconnaît pas
les personnes et peut manquer un objet qui ne coupe pas ce plan. La caméra sert
à lire les flèches, pas à assurer l'arrêt d'urgence. La détection du ruban est
un réglage initial à valider sur les vraies flèches ; ajuster `white_value_min`,
`white_saturation_max`, `roi_top_ratio` et `min_arrow_area_px` dans
`config/autonomy.yaml`. Le masque `/autonomy/arrow_mask` aide à voir ce qui est
détecté.

L'image de la caméra est actuellement lente. La vitesse autonome par défaut est
donc limitée à 0,10 m/s. Vérifier le sens de braquage et les seuils à l'arrêt,
puis roues levées, avant tout essai au sol ; mesurer la distance d'arrêt réelle
à basse vitesse avant d'approcher des personnes ou des murs.

## Clonage comportemental caméra + LiDAR

Un deuxième contrôleur peut apprendre les commandes à partir de démonstrations
humaines. Il reçoit l'image OAK et le secteur avant du LiDAR, puis prédit le
braquage, la vitesse et une classe d'arrêt explicite. Le contrôleur par flèches
blanches reste le défaut. Les deux ne tournent jamais en même temps. Dans les
deux modes, `lidar_safety_gate` reste le dernier maillon avant le VESC.

Le dépôt contient un ancien fichier de poids caméra stéréo, mais il n'est pas
compatible avec la caméra OAK et le LiDAR de ce nœud. Il faut enregistrer des
démonstrations avec la voiture actuelle avant d'activer le nouveau modèle.

### Enregistrer les démonstrations

Sur la Jetson, démarrer `control`, `lidar`, la caméra OAK et le profil
`recording` :

```bash
cd ~/robocar-2026/ros2
docker compose -f docker/compose.yaml --profile control --profile lidar \
  --profile camera_3d_oak --profile recording up -d --build \
  control lidar camera_3d_oak recording
docker compose -f docker/compose.yaml logs -f recording
```

Le collecteur ne publie aucune commande. Il enregistre à 5 Hz l'image, le scan
et la consigne finale `/ackermann_cmd_safe`, y compris les vitesses nulles
produites par les arrêts manuels ou le filet LiDAR. Faire plusieurs passages
à basse vitesse : flèches indiquant tout droit, à gauche et à droite, avec
des formes, positions et orientations différentes, puis des arrêts où le
véhicule doit s'immobiliser.
Pour conduire manuellement, maintenir LB ; garder une personne à la manette et
respecter les consignes de sécurité du README.

Les fichiers sont écrits dans `dataset_behavioral/`. Arrêter l'enregistrement
avec `docker compose -f docker/compose.yaml stop recording`.

### Entraîner et activer

Sur un ordinateur avec PyTorch, NumPy, OpenCV et le paquet Python `onnx` :

```bash
python3 -m pip install torch numpy opencv-python onnx
python3 model/train_ros_behavioral_cloning.py \
  --csv ros2/dataset_behavioral/demonstrations.csv \
  --model-out ros2/models/behavioral_cloning.onnx
```

Le script refuse un jeu de données sans exemples en mouvement et à l'arrêt.
Il affiche la précision et le rappel de la classe d'arrêt sur les 20 % les
plus récents. Vérifier ces résultats et copier/synchroniser `ros2/models/` sur
la Jetson avant l'activation.

Après avoir démarré les autres briques, sélectionner le modèle à la place du
suivi classique :

```bash
ROBOCAR_BEHAVIORAL_CLONING=true docker compose -f docker/compose.yaml \
  --profile autonomy up -d --force-recreate autonomy
```

La vitesse reste plafonnée à 0,10 m/s dans `config/autonomy.yaml`. Si le modèle
ou un capteur manque, le contrôleur publie une consigne nulle. Pour revenir au
suivi classique, recréer `autonomy` sans `ROBOCAR_BEHAVIORAL_CLONING=true`.
Les arrêts appris complètent le LiDAR ; ils ne remplacent pas sa garde finale.
