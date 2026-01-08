#!/bin/bash

# 1. Nettoyage préventif des processus fantômes
echo "🧹 Nettoyage des processus Gazebo..."
killall -9 gzserver gzclient 2> /dev/null

# 2. Nettoyage des dossiers de compilation (Optionnel : commente cette ligne si c'est trop lent)
echo "🗑️  Suppression des anciens fichiers de build..."
rm -rf build/ install/ log/

# 3. Compilation
echo "🔨 Compilation du projet..."
colcon build --symlink-install

# Vérification si la compilation a réussi
if [ $? -ne 0 ]; then
    echo "❌ Erreur de compilation. Arrêt du script."
    exit 1
fi

# 4. Sourcing de l'environnement
echo "🌿 Sourcing de l'environnement..."
source install/setup.bash

# 5. Lancement de la simulation
echo "🚀 Lancement de la simulation..."
ros2 launch in424_simu start_world_launch.py
