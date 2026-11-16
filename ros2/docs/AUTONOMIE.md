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
