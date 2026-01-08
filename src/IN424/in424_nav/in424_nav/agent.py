#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

# --- 1. POINTS DE PASSAGE (RAILS) ---
WAYPOINTS = {
    "Cuisine": (9.0, 1.0),

    # Colonne Vertébrale (ROUGE)
    "R_P1": (9.0, 5.0),   # Entrée Autoroute
    "R_P2": (5.5, 5.0),   # Carrefour Haut (Vers Bleus)
    "R_P3": (5.5, 2.0),   # Descente
    "R_P4": (5.5, -2.0),  # Descente
    "R_P5": (5.5, -5.7),  # Carrefour Bas (Vers Verts)

    # Ligne du Haut (BLEUE) - Y = 5.0
    "B_P1": (3.5, 5.0),
    "B_P2": (0.5, 5.0),
    "B_P3": (-3.5, 5.0),
    "B_P4": (-6.5, 5.0),

    # Ligne du Bas (VERTE) - Y = -5.7
    "V_P1": (2.7, -5.7),
    "V_P2": (0.0, -5.7),
    "V_P3": (-3.5, -5.7),
    "V_P4": (-6.5, -5.7),
}

# --- 2. POINTS DE SERVICE (TABLES) ---
# Coordonnées exactes fournies pour le service
TABLE_SPOTS = {
    # Rangée 1 (Bas - Loin)
    "table_11": (-6.5, -6.5),
    "table_12": (-3.5, -6.5),
    "table_13": (0.0, -6.5),
    "table_14": (2.8, -6.5),

    # Rangée 2 (Bas - Proche)
    "table_21": (-6.5, -4.5),
    "table_22": (-3.5, -4.5),
    "table_23": (0.0, -4.5),
    "table_24": (2.5, -4.5),

    # Rangée 3 (Haut - Proche)
    "table_31": (-6.5, 2.8),
    "table_32": (-3.5, 2.8),
    "table_33": (0.0, 2.8),
    "table_34": (2.5, 2.8),

    # Rangée 4 (Haut - Loin)
    "table_41": (-3.5, 6.5),
    "table_42": (0.5, 6.5),
    "table_43": (3.5, 6.5),
    "table_44": (6.5, 6.5),
}

class RestaurantAgent(Node):
    def __init__(self):
        super().__init__('restaurant_agent')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/bot_1/odom', self.odom_callback, 10)
        
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.current_path = []
        self.path_index = 0
        self.state = "IDLE"
        self.wait_start_time = None
        
        self.linear_speed = 0.6
        self.angular_speed = 1.0
        self.dist_tolerance = 0.15
        self.angle_tolerance = 0.05

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🤖 Robot Serveur Prêt.")

        # --- TEST : CHANGEZ ICI LA TABLE À SERVIR ---
        self.go_to_table("table_44") 

    def get_closest_waypoint(self, target_x, line_points):
        """ Trouve le point de passage dont le X est le plus proche de la table """
        best_wp = None
        min_dist = 999.9
        
        for wp_name in line_points:
            wp_x = WAYPOINTS[wp_name][0]
            dist = abs(target_x - wp_x)
            if dist < min_dist:
                min_dist = dist
                best_wp = wp_name
        return best_wp

    def go_to_table(self, table_name):
        if table_name not in TABLE_SPOTS:
            self.get_logger().error(f"❌ Table inconnue: {table_name}")
            return

        self.get_logger().info(f"🚀 Service demandé : {table_name}")
        
        target_coords = TABLE_SPOTS[table_name]
        table_x, table_y = target_coords
        row = int(table_name[6]) # Récupère le chiffre des dizaines (rangée)

        path = []
        # 1. Sortie de la cuisine (Tronc commun)
        path.append( (WAYPOINTS["Cuisine"], "MOVE") )
        path.append( (WAYPOINTS["R_P1"], "MOVE") )
        path.append( (WAYPOINTS["R_P2"], "MOVE") ) # Carrefour principal (5.5, 5.0)

        exit_waypoint = None

        # 2. Routage par Zone
        if row in [1, 2]: 
            # --- ZONE SUD (VERTE) ---
            self.get_logger().info("   -> Itinéraire : Sud (Vert)")
            # On descend le backbone Rouge
            path.append( (WAYPOINTS["R_P3"], "MOVE") )
            path.append( (WAYPOINTS["R_P4"], "MOVE") )
            path.append( (WAYPOINTS["R_P5"], "MOVE") ) # Arrivée en bas (5.5, -5.7)
            
            # On cherche le point VERT le plus proche en X
            green_points = ["V_P1", "V_P2", "V_P3", "V_P4", "R_P5"]
            exit_name = self.get_closest_waypoint(table_x, green_points)
            exit_waypoint = WAYPOINTS[exit_name]
            
            # Si le point de sortie n'est pas R_P5, on l'ajoute
            if exit_name != "R_P5":
                path.append( (exit_waypoint, "MOVE") )

        elif row in [3, 4]:
            # --- ZONE NORD (BLEUE) ---
            self.get_logger().info("   -> Itinéraire : Nord (Bleu)")
            # On est déjà à R_P2 (5.5, 5.0)
            
            # On cherche le point BLEU (ou Rouge) le plus proche en X
            blue_points = ["B_P1", "B_P2", "B_P3", "B_P4", "R_P2"]
            exit_name = self.get_closest_waypoint(table_x, blue_points)
            exit_waypoint = WAYPOINTS[exit_name]
            
            if exit_name != "R_P2":
                path.append( (exit_waypoint, "MOVE") )

        # 3. Approche Finale (Le point de service exact)
        path.append( (target_coords, "SERVE") )

        # 4. Construction du retour (Miroir)
        return_path = []
        for pt, action in reversed(path[:-1]):
            return_path.append( (pt, "MOVE") )
        
        path.extend(return_path)
        path.append( (WAYPOINTS["Cuisine"], "STOP") )

        self.current_path = path
        self.path_index = 0
        self.state = "MOVING"

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        vel = Twist()
        if self.state == "IDLE" or self.path_index >= len(self.current_path):
            self.cmd_vel_pub.publish(Twist())
            return

        (tx, ty), action = self.current_path[self.path_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx**2 + dy**2)
        angle_diff = self.normalize_angle(math.atan2(dy, dx) - self.yaw)

        if self.state == "MOVING":
            if dist < self.dist_tolerance:
                self.get_logger().info(f"✅ Rejoint : ({tx:.1f}, {ty:.1f})")
                if action == "SERVE":
                    self.state = "SERVING"
                    self.wait_start_time = self.get_clock().now()
                    vel.linear.x = 0.0; vel.angular.z = 0.0
                elif action == "STOP":
                    self.state = "IDLE"
                    self.get_logger().info("🏁 Service terminé.")
                else:
                    self.path_index += 1
            
            elif abs(angle_diff) > self.angle_tolerance:
                vel.linear.x = 0.0
                vel.angular.z = max(min(2.0 * angle_diff, self.angular_speed), -self.angular_speed)
            else:
                vel.linear.x = self.linear_speed
                vel.angular.z = 0.8 * angle_diff

        elif self.state == "SERVING":
            if (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().info("🍽️ Client servi. Retour.")
                self.path_index += 1
                self.state = "MOVING"

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