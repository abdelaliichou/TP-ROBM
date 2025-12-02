
# robm_bridge — Passerelles ROS2 pour les robots ROBM

Ce paquet fournit des nœuds passerelles ROS2 permettant de connecter les robots ROBM
aux topics ROS2. Deux modes de transport sont proposés :

- `udp_bridge` : communication avec le robot via UDP (sockets réseau).
- `zenoh_bridge` : communication via Zenoh (non utilisé en ROBM 2025-2026).


----


## Sujets ROS (topics) — publications et abonnements

La passerelle publie les messages des capteurs du robot et s'abonne aux commandes à destination de ses effecteurs. Les noms de topics sont relatifs au namespace du nœud (par défaut sans préfixe). Voici les principaux topics avec leur type ROS et une description :

### Publications (capteurs)

- `imu` — `sensor_msgs/Imu`
  - Données brutes de l'IMU (gyroscope et accéléromètre). Orientation non fournie.
- `battery` — `sensor_msgs/BatteryState`
  - État de la batterie : tension, courant, pourcentage, statut de charge.
- `tof` — `sensor_msgs/Range`
  - Distance mesurée par le capteur ToF (Time-of-Flight).
- `color` — `robm_interfaces/Color`
  - Valeurs RGB et luminosité (lux) du capteur couleur.

### Abonnements (commandes)

- `cmd_vel` — `geometry_msgs/TwistStamped` (ou `geometry_msgs/Twist`, voir paramètre)
  - Commande de vitesse de la base mobile : `linear.x`, `linear.y`, `angular.z`.
- `cmd_motors` — `robm_interfaces/RoverMotorsCmd`
  - Commande directe des 4 moteurs (valeurs de -1.0 à 1.0).
- `cmd_servo` — `robm_interfaces/ServoCommand`
  - Commande de position du servomoteur (angle en radians, -π/2 à +π/2).
- `cmd_gripper` — `robm_interfaces/GripperCommand`
  - Commande d'ouverture de la pince (0.0 fermé, 1.0 ouvert).


## Paramètres de configuration

Un paramètre commun à tous les nœuds bridge :

- `use_twist_stamped` (booléen, défaut : `True`)
	- Si `True`, le nœud s'abonne à `TwistStamped` sur `cmd_vel` (comportement par défaut depuis ROS2 Jazzy).
	- Sinon, il s'abonne à `Twist` sur `cmd_vel`.

----


## Configuration de `udp_bridge`

Paramètres déclarés dans le nœud `udp_bridge` (et leurs valeurs par défaut) :

- `robot_ip` (chaîne, défaut : `"192.168.4.1"`)
    - Adresse IPv4 du robot (généralement en connexion directe Wi-Fi).
- `robot_port` (entier, défaut : `7448`)
    - Port UDP utilisé par le robot pour les commandes et la télémétrie.

Exemple — lancer avec une IP et un port personnalisés :

```bash
# Lancer avec une adresse robot personnalisée
ros2 run robm_bridge udp_bridge --ros-args -p robot_ip:="192.168.4.2" -p robot_port:=7448
```

Exemple — depuis Pixi (racine du workspace) :

```bash
pixi run ros2 run robm_bridge udp_bridge --ros-args -p robot_ip:="192.168.4.2"
```

Exemple de fichier YAML de paramètres (pour un launch file) :

```yaml
ros__parameters:
  robot_ip: "192.168.4.2"
  robot_port: 7448
  use_twist_stamped: true
```

----


## Configuration de `zenoh_bridge`

Paramètres déclarés dans le nœud `zenoh_bridge` (et leurs valeurs par défaut) :

- `node_id` (chaîne, défaut : `""`, OBLIGATOIRE)
    - Nom logique du robot utilisé pour former les clés Zenoh (ex : `Moulounon`).
    - Ce champ est sensible à la casse et obligatoire (le nœud quitte si vide).
- `use_robot_wifi` (booléen, défaut : `False`)
    - Si `True`, le bridge fonctionne en mode pair-à-pair (Wi-Fi robot, UDP/multicast).
    - Sinon, il se connecte à un routeur Zenoh via `connect_endpoint`.
- `connect_endpoint` (chaîne, défaut : `'tcp/192.168.0.10:7447'`)
    - Adresse du routeur Zenoh à contacter si `use_robot_wifi` est `False`.

Comportement :

- `zenoh_bridge` ouvre une session Zenoh et s'abonne à `robm/robots/<node_id>/sensors`,
  publie sur `robm/robots/<node_id>/effectors` (encodage MessagePack).

Le nœud vérifie que les variables d'environnement limitent ROS2 à l'interface locale :

- `ROS_LOCALHOST_ONLY=1`
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`

Exemple — connexion à un routeur Zenoh :

```bash
# Limiter la découverte ROS2 au localhost
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Connexion à un routeur Zenoh
ros2 run robm_bridge zenoh_bridge --ros-args -p node_id:="Moulounon" -p connect_endpoint:="tcp/192.168.0.10:7447"
```

Exemple — mode Wi-Fi robot (pair-à-pair) :

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 run robm_bridge zenoh_bridge --ros-args -p node_id:="Moulounon" -p use_robot_wifi:=true
```

---


## Bonnes pratiques d'utilisation

1. Construisez et sourcez le workspace (pour le support des messages/services) :

```bash
pixi run build
source install/setup.bash
```

2. Lancez la passerelle souhaitée (`udp_bridge` ou `zenoh_bridge`) avec les bons paramètres.

3. Utilisez `ros2 topic echo` et `ros2 topic pub` pour inspecter les topics capteurs
   et envoyer des commandes de test.


## Dépannage

- Si le message `Failed to open Zenoh session` apparaît, vérifiez que la bibliothèque Python `zenoh` est bien installée dans l'environnement d'exécution.
- Si aucun message capteur n'arrive, vérifiez la connectivité réseau avec le robot (ping, `nc -u`) et la configuration IP.
- Si des erreurs de type support apparaissent (ex : `librobm_interfaces__rosidl_typesupport_*` manquant), assurez-vous que le paquet `robm_interfaces` est bien présent dans le workspace, puis reconstruisez avec `pixi run build` (ou `colcon build ...`) et sourcez `install/setup.bash` avant de lancer les nœuds.