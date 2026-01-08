# On part d'une version officielle de ROS Galactic (basée sur Ubuntu 20.04)
FROM osrf/ros:galactic-desktop

# On évite les questions interactives pendant l'installation
ENV DEBIAN_FRONTEND=noninteractive

# Installation des outils de base et mise à jour
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# Installation des dépendances ROS spécifiques à ton projet (issues de ton script)
RUN apt-get update && apt-get install -y \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-tf-transformations \
    ros-galactic-tf2-tools \
    ros-galactic-teleop-twist-keyboard \
    ros-galactic-xacro \
    && rm -rf /var/lib/apt/lists/*
   

# Installation des dépendances Python
RUN pip3 install transforms3d
RUN pip3 install transforms3d Pillow bresenham

# Configuration automatique de l'environnement ROS à chaque lancement
RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc

# On définit le dossier de travail par défaut
WORKDIR /root/ros2_ws
