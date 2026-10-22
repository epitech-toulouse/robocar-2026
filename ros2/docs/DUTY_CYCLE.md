# Passage du contrôle ROS 2 en duty cycle

Depuis le 2026-10-21, la chaîne `joy_teleop` / `/drive` → mux →
`ackermann_to_vesc` commande `/commands/motor/duty_cycle`.

`drive.speed` conserve son unité nominale m/s pour les nœuds existants, mais la
conversion est **ouverte**, sans régulation de vitesse :

```text
duty = clamp(speed_to_duty_gain × drive.speed, −max_duty_cycle, +max_duty_cycle)
speed_to_duty_gain = 0,10 ; max_duty_cycle = 0,10
```

| Consigne `drive.speed` | Duty envoyé |
|---|---|
| 0 | 0 |
| 0,45 | 0,045 (valeur de l'autopilote Python) |
| 0,5 | 0,05 |
| 1 et au-delà | 0,10 (plafond du collecteur Python manuel) |
| −0,5 | −0,05 |

La vitesse réelle varie avec la batterie et la charge. Le gain n'est pas une
calibration physique. `speed_to_erpm_gain` continue de convertir la télémétrie
eRPM en odométrie ; il ne commande plus le moteur.

## Sécurités conservées

- LB : commande manuelle prioritaire ; RB maintenu : autorisation de `/drive`.
- Le verrou expire après 0,1 s sans autorisation de la manette.
- Sans commande Ackermann pendant 0,1 s : duty zéro, répété à 50 Hz. Publier
  `/drive` à 20–50 Hz ; une boucle calée sur le LiDAR à 10 Hz est trop limite.
- Vitesse ou direction NaN/Inf : duty zéro immédiat.
- Plafond ±0,10 dans le convertisseur **et** dans le pilote VESC ; limites de
  direction 0,20–0,80 inchangées.
- `duty_cycle_only: true` désactive les abonnements moteur en RPM, courant, frein
  et position. Un ancien publisher `/commands/motor/speed` ne peut plus changer le mode.
- Le timeout matériel configuré dans le VESC reste à 500 ms avec freinage 5 A
  en cas de perte du pilote. Aucun heartbeat ne maintient une ancienne consigne.

Duty zéro coupe la traction comme dans les scripts Python ; la distance d'arrêt
peut différer du mode RPM. Les anciens délais mesurés ne valident pas le freinage
physique de ce nouveau mode. Il faut refaire les essais roues en l'air, puis au
sol. Les commandes directes sur le topic moteur contournent toujours le mux :
les programmes de conduite doivent publier sur `/drive`.

## Correction USB

L'envoi est maintenant synchrone et protégé par un mutex jusqu'au dernier octet.
Les écritures partielles sont terminées avant le paquet suivant ; une erreur
ou une écriture sans progression est remontée. Les données d'un paquet temporaire
restent donc valides jusqu'à la fin de sa transmission. Le contrôle de flux USB
est désactivé, comme dans pyserial.

## Vérifier sans voiture

Depuis le dossier `ros2` :

```bash
python3 tests/run_native_tests.py
```

Nécessite Python et un compilateur C++14 avec AddressSanitizer/UndefinedBehaviorSanitizer.
Les tests vérifient les plafonds, le neutre, la marche arrière, NaN/Inf, les
paramètres invalides, les écritures USB partielles/échouées et les paquets concurrents.
Ils sont également enregistrés dans CTest lors du build ROS.

Après reconstruction de l'image, pour tester la chaîne ROS sans manette ni VESC,
arrêter le service `control` réel et lancer dans un terminal :

```bash
docker compose -f docker/compose.yaml stop control
docker run --rm --name robocar-control-test --network host --ipc host \
  --env-file .env -v "$PWD/tests:/tests:ro" robocar:jazzy \
  ros2 launch robocar_bringup control.launch.py with_joy:=false with_vesc:=false
```

Puis dans un autre terminal :

```bash
docker exec robocar-control-test /entrypoint.sh python3 /tests/test_safety_chain.py --kill-mux
```

Résultat attendu : **16/16 OK**. Vérifie notamment RB relâché, manette muette,
perte de `/drive`, priorité LB, plafonds et commandes invalides. Le test détruit
le mux à la fin ; quitter le conteneur de test avant de relancer la voiture.

## Reconstruire et démarrer sur la Jetson

Les sources sont copiées dans l'image Docker : un simple redémarrage ne charge
pas ces modifications. Depuis `~/robocar-2026/ros2`, roues en l'air :

```bash
docker compose -f docker/compose.yaml stop control
docker compose -f docker/compose.yaml build control
docker compose -f docker/compose.yaml --profile control up -d --force-recreate control
docker compose -f docker/compose.yaml logs --tail=50 control
```

Pour observer la commande et vérifier les paramètres effectivement chargés :

```bash
rc() { docker compose -f docker/compose.yaml exec control /entrypoint.sh "$@"; }
rc ros2 topic echo /commands/motor/duty_cycle
# Dans un autre terminal (redéfinir rc) :
rc ros2 param get /vesc_driver_node duty_cycle_only
rc ros2 param get /vesc_driver_node duty_cycle_max
rc ros2 param get /ackermann_to_vesc_node cmd_timeout
```

Attendu : `true`, `0.1`, `0.1`. Vérifier d'abord le neutre, puis LB/stick, le
relâchement, RB et la perte de commande. Contrôler aussi `/sensors/core`
(`fault_code`, courant, tension). La configuration VESC XML n'est pas modifiée.
