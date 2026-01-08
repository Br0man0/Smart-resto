#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

# --- CARTOGRAPHIE DU RESTAURANT (Tes points "Safe") ---
WAYPOINTS = {
    # Base
    "Cuisine": (9.0, 1.0),

    # Couloir du Haut (Rouge) - Y = 5.0
    "R_P1": (9.0, 5.0),
    "R_P2": (6.0, 5.0),
    "R_P3": (3.0, 5.0),
    "R_P4": (-2.0, 5.0),
    "R_P5": (-5.7, 5.0),

    # Rangée Droite (Bleu) - X = 9.0
    "B_P1": (9.0, 2.5),
    "B_P2": (9.0, 0.5),
    "B_P3": (9.0, -3.5),
    "B_P4": (9.0, -6.5),

    # Rangée Gauche (Vert) - X = -5.7
    "V_P1": (-5.7, 3.0),
    "V_P2": (-5.7, 0.0),
    "V_P3": (-5.7, -3.5),
    "V_P4": (-5.7, -6.5),
    
    # Tables (Exemple pour test)
    "Table_Test_Centre": (3.0, 4.0), # Une table proche de R_P3
}

# --- DÉFINITION DE LA MISSION ---
# C'est ici qu'on définit le chemin "Safe".
# Scénario : Le robot part de la cuisine, va au milieu du couloir rouge, livre, et revient.
CURRENT_MISSION = [
    "Cuisine",  # Départ
    "R_P1",     # Monte au couloir rouge
    "R_P2",     # Avance dans le couloir
    "R_P3",     # Continue
    "Table_Test_Centre", # Approche précise (Sortie du rail safe)
    "R_P3",     # Retour sur le rail safe
    "R_P2",     # Retour
    "R_P1",     # Retour
    "Cuisine"   # Rentre à la base
]

class RestaurantAgent(Node):
    def __init__(self):
        super().__init__('restaurant_agent')
        
        # Communication ROS 2 (Namespace bot_1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/bot_1/odom', self.odom_callback, 10)
        
        # État du robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Gestion de la mission
        self.mission_step = 0 # Étape actuelle dans la liste CURRENT_MISSION
        self.state = "MOVING" # États: MOVING, SERVING, END
        self.wait_start_time = None
        
        # Paramètres de mouvement
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.dist_tolerance = 0.15 # Précision de 15cm
        self.angle_tolerance = 0.05

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🤖 Robot Serveur (bot_1) Prêt ! Système de Waypoints chargé.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        vel = Twist()

        # Si la mission est finie
        if self.mission_step >= len(CURRENT_MISSION):
            self.cmd_vel_pub.publish(Twist()) # Stop
            return

        # 1. Identifier la cible actuelle
        target_name = CURRENT_MISSION[self.mission_step]
        
        # Sécurité : Vérifier que le point existe dans le dictionnaire
        if target_name not in WAYPOINTS:
            self.get_logger().error(f"❌ Point inconnu : {target_name}")
            return
            
        target_x, target_y = WAYPOINTS[target_name]

        # 2. Calculs de navigation
        dist_x = target_x - self.x
        dist_y = target_y - self.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        target_angle = math.atan2(dist_y, dist_x)
        angle_diff = self.normalize_angle(target_angle - self.yaw)

        # 3. Machine à états
        if self.state == "MOVING":
            # Affichage de progression (tous les mètres environ pour ne pas spammer)
            # self.get_logger().info(f"Vers {target_name} : Dist={distance:.2f}m")

            if distance < self.dist_tolerance:
                self.get_logger().info(f"✅ Arrivé au waypoint : {target_name}")
                
                # Si c'est une Table (pas un point de passage), on fait une pause "Service"
                if "Table" in target_name:
                    self.state = "SERVING"
                    self.wait_start_time = self.get_clock().now()
                else:
                    # Si c'est juste un point de passage, on enchaîne direct
                    self.mission_step += 1
                
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            
            elif abs(angle_diff) > self.angle_tolerance:
                # Rotation sur place
                vel.linear.x = 0.0
                vel.angular.z = 1.5 * angle_diff
                # Saturation vitesse
                if vel.angular.z > self.angular_speed: vel.angular.z = self.angular_speed
                if vel.angular.z < -self.angular_speed: vel.angular.z = -self.angular_speed
                
            else:
                # Avancer
                vel.linear.x = self.linear_speed
                # Correction d'angle légère en avançant
                vel.angular.z = 0.5 * angle_diff 

        elif self.state == "SERVING":
            now = self.get_clock().now()
            elapsed = (now - self.wait_start_time).nanoseconds / 1e9
            
            if elapsed > 5.0: # Pause de 5 secondes
                self.get_logger().info("🍽️ Service terminé. Retour à la navigation.")
                self.mission_step += 1
                self.state = "MOVING"
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.0

        self.cmd_vel_pub.publish(vel)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def euler_from_quaternion(self, quat):
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