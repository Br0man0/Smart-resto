#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import sys

# --- Configuration ---
# Dictionnaire des points d'intérêt (Menu des destinations)
# Format: "Nom": (x, y)
SERVING_SPOTS = {
    "cuisine": (9.0, 1.0),   # Point de départ (Spawn)
    "table_5": (0.03, -2.6), # Exemple de table centrale
    # Tu pourras ajouter les autres tables ici
}

class RestaurantAgent(Node):
    def __init__(self):
        super().__init__('restaurant_agent')
        
        # 1. Initialisation des Publishers / Subscribers
        # Pour envoyer les commandes de vitesse
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Pour écouter la position du robot (Odométrie)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Pour écouter le LIDAR (Scanner) - Utile plus tard pour les obstacles
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # 2. Variables d'état du robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # Angle (orientation) actuel en radians
        self.lidar_ranges = [] # Données du laser

        # 3. Logique de navigation
        self.current_goal = None        # Tuple (x, y) de la destination actuelle
        self.state = "IDLE"             # États: IDLE, ROTATING, MOVING
        
        # Timer : La boucle principale de contrôle (exécutée 10 fois par seconde)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("🤖 Robot Serveur Prêt ! En attente d'ordres...")

    def odom_callback(self, msg):
        """
        Met à jour la position (x, y) et l'orientation (yaw) du robot
        à chaque fois que ROS envoie une nouvelle donnée d'odométrie.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Conversion du Quaternion (ROS) en Angle Euler (Maths)
        orientation_q = msg.pose.pose.orientation
        self.yaw = self.euler_from_quaternion(orientation_q)

    def scan_callback(self, msg):
        """
        Stocke les distances mesurées par le LIDAR.
        """
        self.lidar_ranges = msg.ranges

    def control_loop(self):
        """
        Cerveau du robot. Cette fonction est appelée en boucle.
        C'est ici que nous déciderons des mouvements.
        """
        # Création du message de vitesse (par défaut à 0)
        vel_msg = Twist()

        # --- LOGIQUE À IMPLÉMENTER ICI DANS LA PROCHAINE ÉTAPE ---
        # Pour l'instant, on affiche juste la position pour vérifier que tout marche
        # self.get_logger().info(f"Position: x={self.x:.2f}, y={self.y:.2f}, angle={self.yaw:.2f}")

        # Envoi de la commande au robot
        self.cmd_vel_pub.publish(vel_msg)

    # --- Fonctions Utilitaires ---

    def euler_from_quaternion(self, quat):
        """
        Convertit un quaternion (x, y, z, w) en angle lacet (yaw) autour de l'axe Z.
        C'est des maths pures, pas besoin d'y toucher.
        """
        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(t3, t4)

def main(args=None):
    rclpy.init(args=args)
    agent = RestaurantAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()