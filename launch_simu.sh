#!/bin/bash

echo "🧹 Nettoyage..."
killall -9 gzserver gzclient python3 2> /dev/null

echo "🔨 Compilation..."
colcon build --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Erreur de compilation."
    exit 1
fi

echo "🌿 Sourcing..."
source install/setup.bash

# Lancement en arrière-plan (&)
echo "🚀 Simulation..."
ros2 launch in424_simu start_world_launch.py &
sleep 5 # Attendre que Gazebo charge un peu

echo "🧠 Lancement du Cerveau (Agent)..."
ros2 run in424_nav agent &

echo "🖥️  Lancement de l'Interface Graphique..."
ros2 run in424_nav gui