import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    simu_pkg = get_package_share_directory("in424_simu")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    # Ajout du chemin des modèles au path Gazebo
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + os.path.join(simu_pkg, "models")
    else:
        os.environ["GAZEBO_MODEL_PATH"] = os.path.join(simu_pkg, "models")

    # Définition du lancement de Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        )
    )

    # Définition du spawn des robots
    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, "launch", "spawn_robots_launch.py")
        )
    )

    return LaunchDescription([
        # Argument pour choisir le monde (par défaut env.world)
        DeclareLaunchArgument(
            "world",
            default_value = os.path.join(simu_pkg, "worlds", "env.world"),
            description = "World file to use for the simulation"
        ),

        # 1. On lance Gazebo directement
        gazebo_launch,

        # 2. On attend 5 secondes que Gazebo soit prêt, puis on fait apparaître le robot
        TimerAction(
           period = 5.0,
           actions = [
                spawn_robots_launch
           ]
        ),

        # 3. On lance RViz
        Node(
            package = "rviz2",
            executable = "rviz2",
            arguments = ["-d", os.path.join(simu_pkg, "cfg", "config.rviz")]
        )
    ])