#!/bin/bash

# 1. Charger l'environnement ROS 2 Galactic global
source /opt/ros/galactic/setup.bash

# 2. Charger les variables Gazebo (évite les bugs de modèles manquants)
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# 3. Charger ton Workspace local
# Le script cherche le dossier install où qu'il soit par rapport au home
WS_DIR="$HOME/ros2_ws"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "ERREUR : Fichier install/setup.bash introuvable !"
    echo "As-tu bien lancé 'colcon build' ?"
    exit 1
fi

# 4. Lancer la simulation
echo "🚀 Lancement de la simulation IN424..."
ros2 launch in424_simu start_world_launch.py
