#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

# --- 1. CARTOGRAPHIE ---
WAYPOINTS = {
    "Cuisine": (9.0, 1.0),
    "R_P1": (9.0, 5.0), "R_P2": (5.5, 5.0), "R_P3": (5.5, 2.0),
    "R_P4": (5.5, -2.0), "R_P5": (5.5, -5.7),
    "B_P1": (3.5, 5.0), "B_P2": (0.5, 5.0), "B_P3": (-3.5, 5.0), "B_P4": (-6.5, 5.0),
    "V_P1": (2.7, -5.7), "V_P2": (0.0, -5.7), "V_P3": (-3.5, -5.7), "V_P4": (-6.5, -5.7),
}

TABLE_SPOTS = {
    "table_11": (-6.5, -6.5), "table_12": (-3.5, -6.5), "table_13": (0.0, -6.5), "table_14": (2.8, -6.5),
    "table_21": (-6.5, -4.5), "table_22": (-3.5, -4.5), "table_23": (0.0, -4.5), "table_24": (2.5, -4.5),
    "table_31": (-6.5, 2.8), "table_32": (-3.5, 2.8), "table_33": (0.0, 2.8), "table_34": (2.5, 2.8),
    "table_41": (-3.5, 6.5), "table_42": (0.5, 6.5), "table_43": (3.5, 6.5), "table_44": (6.5, 6.5),
}

class RestaurantAgent(Node):
    def __init__(self):
        super().__init__('restaurant_agent')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/bot_1/odom', self.odom_callback, 10)
        self.gui_sub = self.create_subscription(String, '/gui/command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, '/gui/status', 10)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.current_path = []
        self.path_index = 0
        self.state = "IDLE"
        self.return_signal_received = False
        
        self.linear_speed = 0.6
        self.angular_speed = 1.0
        self.dist_tolerance = 0.15
        self.angle_tolerance = 0.05

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🤖 NOUVELLE VERSION CHARGÉE : Attente active activée.")

    def command_callback(self, msg):
        command = msg.data
        if command == "RETURN":
            if self.state == "SERVING":
                self.get_logger().info("👍 RETOUR CONFIRMÉ PAR L'INTERFACE.")
                self.return_signal_received = True
            else:
                self.get_logger().warn(f"Commande RETURN ignorée (État actuel: {self.state})")
        elif command in TABLE_SPOTS:
            if self.state == "IDLE":
                self.get_logger().info(f"🚀 Départ vers {command}")
                self.go_to_table(command)
            else:
                self.get_logger().warn("Robot occupé.")

    def get_closest_waypoint(self, target_x, line_points):
        best_wp = None; min_dist = 999.9
        for wp_name in line_points:
            dist = abs(target_x - WAYPOINTS[wp_name][0])
            if dist < min_dist: min_dist = dist; best_wp = wp_name
        return best_wp

    def go_to_table(self, table_name):
        self.return_signal_received = False
        target_coords = TABLE_SPOTS[table_name]
        row = int(table_name[6])
        
        path = []
        path.append((WAYPOINTS["Cuisine"], "MOVE"))
        path.append((WAYPOINTS["R_P1"], "MOVE"))
        path.append((WAYPOINTS["R_P2"], "MOVE"))

        if row in [1, 2]: 
            path.append((WAYPOINTS["R_P3"], "MOVE"))
            path.append((WAYPOINTS["R_P4"], "MOVE"))
            path.append((WAYPOINTS["R_P5"], "MOVE"))
            green_points = ["V_P1", "V_P2", "V_P3", "V_P4", "R_P5"]
            exit_name = self.get_closest_waypoint(target_coords[0], green_points)
            if exit_name != "R_P5": path.append((WAYPOINTS[exit_name], "MOVE"))
        elif row in [3, 4]:
            blue_points = ["B_P1", "B_P2", "B_P3", "B_P4", "R_P2"]
            exit_name = self.get_closest_waypoint(target_coords[0], blue_points)
            if exit_name != "R_P2": path.append((WAYPOINTS[exit_name], "MOVE"))

        path.append((target_coords, "SERVE"))

        return_path = []
        for pt, action in reversed(path[:-1]): return_path.append((pt, "MOVE"))
        path.extend(return_path)
        path.append((WAYPOINTS["Cuisine"], "STOP"))

        self.current_path = path
        self.path_index = 0
        self.state = "MOVING"

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        status_msg = String()
        status_msg.data = self.state
        self.status_pub.publish(status_msg)

        vel = Twist()
        
        if self.state == "IDLE" or self.path_index >= len(self.current_path):
            self.cmd_vel_pub.publish(Twist())
            return

        (tx, ty), action = self.current_path[self.path_index]
        dx = tx - self.x; dy = ty - self.y
        dist = math.sqrt(dx**2 + dy**2)
        angle_diff = self.normalize_angle(math.atan2(dy, dx) - self.yaw)

        if self.state == "MOVING":
            if dist < self.dist_tolerance:
                if action == "SERVE":
                    self.state = "SERVING"
                    self.get_logger().info("🍽️ Arrivé à table. EN ATTENTE DU BOUTON...")
                    vel.linear.x = 0.0; vel.angular.z = 0.0
                elif action == "STOP":
                    self.state = "IDLE"
                    self.get_logger().info("🏁 Mission terminée.")
                else:
                    self.path_index += 1
            elif abs(angle_diff) > self.angle_tolerance:
                vel.linear.x = 0.0
                vel.angular.z = max(min(2.0 * angle_diff, self.angular_speed), -self.angular_speed)
            else:
                vel.linear.x = self.linear_speed
                vel.angular.z = 0.8 * angle_diff

        elif self.state == "SERVING":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            
            if self.return_signal_received:
                self.get_logger().info("✅ Signal reçu. On rentre !")
                self.path_index += 1
                self.state = "MOVING"
                self.return_signal_received = False

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
    try: rclpy.spin(agent)
    except KeyboardInterrupt: pass
    finally: agent.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()