import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion

import numpy as np

from .my_common import *    #common variables are stored here

from bresenham import bresenham

OCCUPIED_SPACE_VALUE = 100
FREE_SPACE_VALUE = 0
import random

class MonteCarloLocalization:
    def __init__(self, num_particles, map_size, robot_size):
        self.num_particles = num_particles
        self.map_size = map_size
        self.robot_size = robot_size
        self.particles = self.initialize_particles()
        self.weights = np.ones(num_particles) / num_particles
        self.resample_threshold = 0.5 * num_particles  # Pour le resampling adaptatif
       
    def initialize_particles(self):
        """ Initialiser les particules uniformément sur la carte """
        return [(random.uniform(-self.map_size[1]/2, self.map_size[1]/2),
                 random.uniform(-self.map_size[0]/2, self.map_size[0]/2),
                 random.uniform(-np.pi, np.pi))
                for _ in range(self.num_particles)]
   
    def motion_update(self, delta_x, delta_y, delta_yaw):
        """ Mettre à jour les particules selon le modèle de mouvement avec bruit """
        # Ajuster les paramètres de bruit selon les caractéristiques du robot
        noise_x = 0.05 * abs(delta_x) + 0.01
        noise_y = 0.05 * abs(delta_y) + 0.01
        noise_yaw = 0.1 * abs(delta_yaw) + 0.01
       
        self.particles = [(x + delta_x + np.random.normal(0, noise_x),
                           y + delta_y + np.random.normal(0, noise_y),
                           self.normalize_angle(yaw + delta_yaw + np.random.normal(0, noise_yaw)))
                          for (x, y, yaw) in self.particles]
   
    def normalize_angle(self, angle):
        """ Normaliser un angle entre -pi et pi """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
   
    def measurement_update(self, lidar_readings, current_map, robot_x, robot_y, robot_yaw, map_info):
        """ Mettre à jour les poids des particules avec les lectures LIDAR """
        angles = np.linspace(lidar_readings.angle_min, lidar_readings.angle_max, len(lidar_readings.ranges))
       
        for i, (px, py, pyaw) in enumerate(self.particles):
            # Calculer la probabilité que cette particule soit à la bonne position
            likelihood = 1.0
           
            # Échantillonner quelques rayons du LIDAR pour une comparaison efficace
            sampling_rate = 10  # Utiliser 1 rayon sur 10
            for j in range(0, len(lidar_readings.ranges), sampling_rate):
                if lidar_readings.ranges[j] >= lidar_readings.range_max:
                    continue  # Ignorer les mesures hors de portée
               
                # Angle du rayon dans le référentiel global
                global_angle = pyaw + angles[j]
               
                # Point observé par le LIDAR depuis la position du robot
                observed_x = robot_x + lidar_readings.ranges[j] * np.cos(robot_yaw + angles[j])
                observed_y = robot_y + lidar_readings.ranges[j] * np.sin(robot_yaw + angles[j])
               
                # Point attendu si le robot était à la position de la particule
                expected_x = px + lidar_readings.ranges[j] * np.cos(global_angle)
                expected_y = py + lidar_readings.ranges[j] * np.sin(global_angle)
               
                # Convertir en indices de carte
                map_width = map_info.width
                map_height = map_info.height
                resolution = map_info.resolution
                origin_x = map_info.origin.position.x
                origin_y = map_info.origin.position.y
               
                # Indices de la carte pour le point observé
                obs_grid_x = int((observed_x - origin_x) / resolution)
                obs_grid_y = map_height - int((observed_y - origin_y) / resolution)
               
                # Indices de la carte pour le point attendu
                exp_grid_x = int((expected_x - origin_x) / resolution)
                exp_grid_y = map_height - int((expected_y - origin_y) / resolution)
               
                # Vérifier si les points sont dans les limites de la carte
                if (0 <= obs_grid_x < map_width and 0 <= obs_grid_y < map_height and
                    0 <= exp_grid_x < map_width and 0 <= exp_grid_y < map_height):
                   
                    # Valeur observée dans la carte
                    obs_value = current_map[obs_grid_y, obs_grid_x]
                   
                    # Valeur attendue dans la carte
                    exp_value = current_map[exp_grid_y, exp_grid_x]
                   
                    # Calculer la similitude entre les valeurs observées et attendues
                    if obs_value == OCCUPIED_SPACE_VALUE and exp_value == OCCUPIED_SPACE_VALUE:
                        # Les deux sont des obstacles - forte probabilité
                        ray_likelihood = 0.9
                    elif obs_value == FREE_SPACE_VALUE and exp_value == FREE_SPACE_VALUE:
                        # Les deux sont libres - bonne probabilité
                        ray_likelihood = 0.7
                    else:
                        # Différence - faible probabilité
                        ray_likelihood = 0.1
                   
                    likelihood *= ray_likelihood
           
            # Mettre à jour le poids de la particule
            self.weights[i] *= likelihood
       
        # Normaliser les poids
        weight_sum = np.sum(self.weights)
        if weight_sum > 0:
            self.weights /= weight_sum
        else:
            # Si tous les poids sont proches de zéro, réinitialiser
            self.weights = np.ones(self.num_particles) / self.num_particles
       
        # Resampling adaptatif
        if self.need_resample():
            self.resample()
   
    def need_resample(self):
        """ Détermine si un resampling est nécessaire """
        # Calcul de la taille effective d'échantillon
        if np.sum(self.weights) == 0:
            return True
        neff = 1.0 / np.sum(np.square(self.weights))
        return neff < self.resample_threshold
   
    def resample(self):
        """ Rééchantillonner les particules selon leurs poids """
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = [self.particles[i] for i in indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
   
    def get_best_estimate(self):
        """ Renvoyer l'estimation de position basée sur les particules pondérées """
        if np.sum(self.weights) == 0:
            # Si tous les poids sont nuls, utiliser la moyenne simple
            x_mean = np.mean([p[0] for p in self.particles])
            y_mean = np.mean([p[1] for p in self.particles])
           
            # Pour l'angle, utiliser la moyenne circulaire
            cos_sum = np.sum([np.cos(p[2]) for p in self.particles])
            sin_sum = np.sum([np.sin(p[2]) for p in self.particles])
            yaw_mean = np.arctan2(sin_sum, cos_sum)
        else:
            # Utiliser la moyenne pondérée
            x_mean = np.sum([p[0] * w for p, w in zip(self.particles, self.weights)])
            y_mean = np.sum([p[1] * w for p, w in zip(self.particles, self.weights)])
           
            # Moyenne circulaire pondérée pour l'angle
            cos_sum = np.sum([np.cos(p[2]) * w for p, w in zip(self.particles, self.weights)])
            sin_sum = np.sum([np.sin(p[2]) * w for p, w in zip(self.particles, self.weights)])
            yaw_mean = np.arctan2(sin_sum, cos_sum)
           
        return x_mean, y_mean, yaw_mean

class Agent(Node):
    """
    This class is used to define the behavior of ONE agent
    """
    def __init__(self):
        Node.__init__(self, "Agent")
        self.load_params()
        
        # --- INITIALISATION ROS STANDARD ---
        self.agents_pose = [None]*self.nb_agents 
        self.x = self.y = self.yaw = None
        self.prev_x = self.prev_y = self.prev_yaw = None
        self.lidar_data = None

        # Publishers / Subscribers essentiels
        self.map_agent_pub = self.create_publisher(OccupancyGrid, f"/{self.ns}/map", 1)
        self.init_map()
        
        # On garde MCL (Localisation) car c'est utile pour se recaler
        self.mcl = MonteCarloLocalization(num_particles=1000, map_size=self.env_size, robot_size=self.robot_size)

        # Abonnements Odométrie
        odom_methods_cb = [self.odom1_cb, self.odom2_cb, self.odom3_cb]
        for i in range(1, self.nb_agents + 1):  
            self.create_subscription(Odometry, f"/bot_{i}/odom", odom_methods_cb[i-1], 1)
        
        if self.nb_agents != 1:
            self.create_subscription(OccupancyGrid, "/merged_map", self.merged_map_cb, 1)
        
        # Capteurs et Actionneurs
        self.create_subscription(LaserScan, f"{self.ns}/laser/scan", self.lidar_cb, qos_profile=qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, f"{self.ns}/cmd_vel", 1)

        # Timers (Boucles de contrôle)
        self.create_timer(0.2, self.map_update) 
        self.create_timer(0.1, self.strategy)      # Accéléré à 10Hz (0.1s) pour un mouvement plus fluide
        self.create_timer(1, self.publish_maps)

        # --- VARIABLES SMART RESTO (NOUVEAU) ---
        self.state = "IDLE"  # États: IDLE, MOVING, ALIGNING, SERVING, RETURNING
        self.serving_timer = 0
        self.current_goal_name = None
        self.target_reached_threshold = 0.05 # Précision de 5cm

        # MENU DES DESTINATIONS (Généré depuis env.world)
        # Format: 'nom': {'x': float, 'y': float, 'yaw': float}
        self.serving_spots = {
            'cuisine': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},

            # --- COULOIR GAUCHE (Entre x=-6.7 et x=-3.5) ---
            'table_1':  {'x': -5.85, 'y': -9.42, 'yaw': 3.14},
            'table_7':  {'x': -5.73, 'y': -2.65, 'yaw': 3.14},
            'table_8':  {'x': -5.66, 'y':  0.03, 'yaw': 3.14},
            'table_11': {'x': -5.61, 'y':  1.07, 'yaw': 3.14},

            'table_2':  {'x': -4.35, 'y': -8.58, 'yaw': 0.0},
            'table_6':  {'x': -4.38, 'y': -2.64, 'yaw': 0.0},
            'table_10': {'x': -4.44, 'y':  1.15, 'yaw': 0.0},
            'table_12': {'x': -4.38, 'y':  8.66, 'yaw': 0.0},

            # --- COULOIR CENTRAL (Entre x=0.0 et x=3.0) ---
            'table_3':  {'x':  0.87, 'y': -8.51, 'yaw': 3.14},
            'table_5':  {'x':  0.89, 'y': -2.61, 'yaw': 3.14},
            'table_9':  {'x':  0.97, 'y':  0.92, 'yaw': 3.14},
            'table_13': {'x':  1.34, 'y':  8.39, 'yaw': 3.14},

            'table_16': {'x':  1.99, 'y': -8.42, 'yaw': 0.0},
            'table_15': {'x':  2.19, 'y': -2.78, 'yaw': 0.0},
            'table_14': {'x':  2.23, 'y':  0.74, 'yaw': 0.0},
            'table_17': {'x':  2.59, 'y':  8.47, 'yaw': 0.0},

            # --- EXTRÊME DROITE ---
            'table_18': {'x':  5.64, 'y':  8.46, 'yaw': 0.0},
            'table_4':  {'x':  6.59, 'y': -8.54, 'yaw': 0.0},
        }

        self.get_logger().info("Smart Resto Agent Initialized & Ready to Serve!")
   

    def move_to_point(self, target_x, target_y):
        """ Déplace le robot vers un point (x,y). Retourne True si arrivé. """
        # Calcul de la distance restante
        dist_error = np.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        
        # Si on est assez près (moins de 5cm), on s'arrête
        if dist_error < self.target_reached_threshold:
            self.cmd_vel_pub.publish(Twist()) # Stop
            return True
            
        # Calcul de l'angle vers la cible
        target_yaw = np.arctan2(target_y - self.y, target_x - self.x)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)
        
        cmd = Twist()
        
        # Logique de navigation simple :
        # 1. Si on regarde trop ailleurs (> 20 degrés), on tourne sur place
        if abs(yaw_error) > 0.35: 
            cmd.angular.z = 0.5 if yaw_error > 0 else -0.5
            cmd.linear.x = 0.0
        else:
            # 2. Sinon, on avance en ajustant la direction
            # On ralentit quand on approche du but (Proportionnel à la distance)
            speed = min(0.5, dist_error * 0.5) 
            cmd.linear.x = max(0.1, speed) # Vitesse min 0.1
            cmd.angular.z = yaw_error * 0.8 # Correction angulaire
            
        self.cmd_vel_pub.publish(cmd)
        return False

    def rotate_to_orientation(self, target_yaw):
        """ Tourne le robot pour atteindre un angle précis. Retourne True si aligné. """
        yaw_error = self.normalize_angle(target_yaw - self.yaw)
        
        # Si l'erreur est très petite (< 3 degrés), on considère que c'est bon
        if abs(yaw_error) < 0.05:
            self.cmd_vel_pub.publish(Twist()) # Stop
            return True
            
        cmd = Twist()
        cmd.linear.x = 0.0
        # Vitesse de rotation fixe mais douce
        cmd.angular.z = 0.3 if yaw_error > 0 else -0.3
        
        self.cmd_vel_pub.publish(cmd)
        return False

    def load_params(self):
        """ Load parameters from launch file """
        self.declare_parameters(    #A node has to declare ROS parameters before getting their values from launch files
            namespace="",
            parameters=[
                ("ns", rclpy.Parameter.Type.STRING),    #robot's namespace: either 1, 2 or 3
                ("robot_size", rclpy.Parameter.Type.DOUBLE),    #robot's diameter in meter
                ("env_size", rclpy.Parameter.Type.INTEGER_ARRAY),   #environment dimensions (width height)
                ("nb_agents", rclpy.Parameter.Type.INTEGER),    #total number of agents (this agent included) to map the environment
            ]
        )

        #Get launch file parameters related to this node
        self.ns = self.get_parameter("ns").value
        self.robot_size = self.get_parameter("robot_size").value
        self.env_size = self.get_parameter("env_size").value
        self.nb_agents = self.get_parameter("nb_agents").value
   

    def init_map(self):
        """ Initialize the map to share with others if it is bot_1 """
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"    #set in which reference frame the map will be expressed (DO NOT TOUCH)
        self.map_msg.header.stamp = self.get_clock().now().to_msg() #get the current ROS time to send the msg
        self.map_msg.info.resolution = self.robot_size  #Map cell size corresponds to robot size
        self.map_msg.info.height = int(self.env_size[0]/self.map_msg.info.resolution)   #nb of rows
        self.map_msg.info.width = int(self.env_size[1]/self.map_msg.info.resolution)    #nb of columns
        self.map_msg.info.origin.position.x = -self.env_size[1]/2   #x and y coordinates of the origin in map reference frame
        self.map_msg.info.origin.position.y = -self.env_size[0]/2
        self.map_msg.info.origin.orientation.w = 1.0    #to have a consistent orientation in quaternion: x=0, y=0, z=0, w=1 for no rotation
        self.map = np.ones(shape=(self.map_msg.info.height, self.map_msg.info.width), dtype=np.int8)*UNEXPLORED_SPACE_VALUE #all the cells are unexplored initially
        self.w, self.h = self.map_msg.info.width, self.map_msg.info.height  
   

    def merged_map_cb(self, msg):
        """
            Get the current common map and update ours accordingly.
            This method is automatically called whenever a new message is published on the topic /merged_map.
            'msg' is a nav_msgs/msg/OccupancyGrid message.
        """
        received_map = np.flipud(np.array(msg.data).reshape(self.h, self.w))    #convert the received list into a 2D array and reverse rows
        for i in range(self.h):
            for j in range(self.w):
                if (self.map[i, j] == UNEXPLORED_SPACE_VALUE) and (received_map[i, j] != UNEXPLORED_SPACE_VALUE):
                # if received_map[i, j] != UNEXPLORED_SPACE_VALUE:
                    self.map[i, j] = received_map[i, j]


    def odom1_cb(self, msg):
        """
            @brief Get agent 1 position.
            This method is automatically called whenever a new message is published on topic /bot_1/odom.
           
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 1:
            # Enregistrer la position précédente pour calculer le déplacement
            if self.x is not None:
                self.prev_x, self.prev_y, self.prev_yaw = self.x, self.y, self.yaw
           
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
           
            # Mise à jour des particules avec le déplacement
            if self.prev_x is not None:
                delta_x = self.x - self.prev_x
                delta_y = self.y - self.prev_y
                delta_yaw = self.normalize_angle(self.yaw - self.prev_yaw)
               
                # Mettre à jour les particules avec le mouvement
                self.mcl.motion_update(delta_x, delta_y, delta_yaw)
       
        self.agents_pose[0] = (x, y)
   

    def odom2_cb(self, msg):
        """
            @brief Get agent 2 position.
            This method is automatically called whenever a new message is published on topic /bot_2/odom.
           
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 2:
            # Enregistrer la position précédente pour calculer le déplacement
            if self.x is not None:
                self.prev_x, self.prev_y, self.prev_yaw = self.x, self.y, self.yaw
           
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
           
            # Mise à jour des particules avec le déplacement
            if self.prev_x is not None:
                delta_x = self.x - self.prev_x
                delta_y = self.y - self.prev_y
                delta_yaw = self.normalize_angle(self.yaw - self.prev_yaw)
               
                # Mettre à jour les particules avec le mouvement
                self.mcl.motion_update(delta_x, delta_y, delta_yaw)
       
        self.agents_pose[1] = (x, y)


    def odom3_cb(self, msg):
        """
            @brief Get agent 3 position.
            This method is automatically called whenever a new message is published on topic /bot_3/odom.
           
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 3:
            # Enregistrer la position précédente pour calculer le déplacement
            if self.x is not None:
                self.prev_x, self.prev_y, self.prev_yaw = self.x, self.y, self.yaw
           
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
           
            # Mise à jour des particules avec le déplacement
            if self.prev_x is not None:
                delta_x = self.x - self.prev_x
                delta_y = self.y - self.prev_y
                delta_yaw = self.normalize_angle(self.yaw - self.prev_yaw)
               
                # Mettre à jour les particules avec le mouvement
                self.mcl.motion_update(delta_x, delta_y, delta_yaw)
       
        self.agents_pose[2] = (x, y)


    def normalize_angle(self, angle):
        """Normalise un angle entre -pi et pi"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


    def map_update(self):
        """ Update the occupancy grid with sensor data """
        self.map_msg.data = np.flipud(self.map).flatten().tolist()
   

    def lidar_cb(self, msg):
        """ Process LIDAR data to detect obstacles and update the map. """
        if self.x is None or self.y is None:
            return  # Skip if agent pose is not yet received
       
        # Stocker les données LIDAR pour les utiliser dans la mise à jour MCL
        self.lidar_data = msg
       
        # Mise à jour de l'estimation Monte Carlo
        if self.x is not None and self.y is not None and self.yaw is not None:
            self.mcl.measurement_update(msg, self.map, self.x, self.y, self.yaw, self.map_msg.info)
       
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        for angle, distance in zip(angles, msg.ranges):
            if distance < msg.range_max:  # Valid obstacle detection
                # Transform to global coordinates
                obs_x = self.x + distance * np.cos(self.yaw + angle)
                obs_y = self.y + distance * np.sin(self.yaw + angle)
               
                # Convert to grid coordinates
                grid_x = int((obs_x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
                grid_y = self.h - int((obs_y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
               
                # Position du robot dans la grille
                robot_x = int((self.x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
                robot_y = self.h - int((self.y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)

                # Tracer une ligne de (robot_x, robot_y) jusqu'à (grid_x, grid_y)
                for free_x, free_y in bresenham(robot_x, robot_y, grid_x, grid_y):
                    if 0 <= free_x < self.w and 0 <= free_y < self.h:
                        self.map[free_y, free_x] = FREE_SPACE_VALUE  # Marquer comme libre

                # Si l'obstacle est dans la grille, le marquer comme occupé
                if 0 <= grid_x < self.w and 0 <= grid_y < self.h:
                    self.map[grid_y, grid_x] = OCCUPIED_SPACE_VALUE  # Marquer comme obstacle


    def publish_maps(self):
        """
            Publish updated map to topic /bot_x/map, where x is either 1, 2 or 3.
            This method is called periodically (1Hz) by a ROS2 timer, as defined in the constructor of the class.
        """
        self.map_msg.data = np.flipud(self.map).flatten().tolist()  #transform the 2D array into a list to publish it
        self.map_agent_pub.publish(self.map_msg)    #publish map to other agents


    def strategy(self):
        """ Machine à états du Smart Resto """
        if self.x is None or self.y is None:
            return # On attend la première lecture de position

        # --- SÉCURITÉ (LIDAR) ---
        # Si obstacle à moins de 40cm devant, arrêt immédiat
        if self.check_obstacle_ahead(dist_threshold=0.4): 
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn("Obstacle détecté ! Pause.")
            return

        # --- MACHINE À ÉTATS ---
        
        if self.state == "IDLE":
            # SCÉNARIO DE TEST : On va à la Table 5
            self.current_goal_name = 'table_5'
            self.get_logger().info(f"--- DÉBUT MISSION : Aller à {self.current_goal_name} ---")
            self.state = "MOVING"

        elif self.state == "MOVING":
            # Étape 1 : Naviguer jusqu'au point de service (dans l'allée)
            target = self.serving_spots[self.current_goal_name]
            finished = self.move_to_point(target['x'], target['y'])
            
            if finished:
                self.get_logger().info("Point atteint. Alignement vers la table...")
                self.state = "ALIGNING"

        elif self.state == "ALIGNING":
            # Étape 2 : Pivoter pour faire face à la table
            target = self.serving_spots[self.current_goal_name]
            aligned = self.rotate_to_orientation(target['yaw'])
            
            if aligned:
                self.get_logger().info("Robot en place. Service en cours...")
                self.serving_timer = self.get_clock().now()
                self.state = "SERVING"

        elif self.state == "SERVING":
            # Étape 3 : Attendre 5 secondes (Simulation interaction client)
            now = self.get_clock().now()
            elapsed = (now - self.serving_timer).nanoseconds / 1e9
            
            if elapsed > 5.0:
                self.get_logger().info("Service terminé. Retour en cuisine.")
                self.current_goal_name = 'cuisine'
                self.state = "RETURNING"

        elif self.state == "RETURNING":
            # Étape 4 : Retour au point (0,0)
            target = self.serving_spots['cuisine']
            
            # On navigue vers la cuisine
            finished = self.move_to_point(target['x'], target['y'])
            
            if finished:
                self.get_logger().info("Retour effectué. En attente.")
                # Pour le test, on arrête là, ou on relance une autre table
                # self.state = "IDLE" (Boucle infinie si on remet IDLE)
                self.state = "DONE" # État final pour stopper

        elif self.state == "DONE":
            self.cmd_vel_pub.publish(Twist()) # S'assurer qu'on est à l'arrêt

    def is_target_reached(self):
        """ Vérifie si la cible actuelle a été atteinte """
        if self.current_target is None:
            return True
           
        target_x, target_y = self.current_target
        distance = np.sqrt((self.x - target_x)**2 + (self.y - target_y)**2)
        return distance < self.target_reached_threshold


    def check_obstacle_ahead(self, dist_threshold=0.5):
        """ Vérifie s'il y a un obstacle devant le robot """
        if self.lidar_data is None:
            return False
            
        # On regarde dans un cône de 60 degrés devant (-30° à +30°)
        front_angle_width = np.pi / 3 
        
        angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(self.lidar_data.ranges))
        
        for i, angle in enumerate(angles):
            if abs(angle) < front_angle_width / 2:
                # Si un rayon détecte quelque chose de proche
                if self.lidar_data.ranges[i] < dist_threshold:
                    return True
        return False


    def find_exploration_target(self):
        """ Trouve une cible d'exploration intelligente """
        # Convertir la position du robot en coordonnées de grille
        robot_x = int((self.x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        robot_y = self.h - int((self.y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
       
        # Trouver toutes les cellules inexplorées
        unexplored_cells = np.argwhere(self.map == UNEXPLORED_SPACE_VALUE)
       
        if len(unexplored_cells) == 0:
            # Si tout est exploré, trouver une partie moins explorée
            free_cells = np.argwhere(self.map == FREE_SPACE_VALUE)
            if len(free_cells) == 0:
                return self.x, self.y  # Si rien n'est exploré, rester en place
               
            # Choisir une cellule libre aléatoire
            target_idx = np.random.randint(0, len(free_cells))
            y, x = free_cells[target_idx]
        else:
            # Identifier les cellules frontières (proches d'espaces libres)
            frontier_cells = []
           
            for cell in unexplored_cells:
                y, x = cell
                # Vérifier si c'est une cellule frontière (proche d'une zone explorée)
                is_frontier = False
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < self.h and 0 <= nx < self.w and
                            self.map[ny, nx] == FREE_SPACE_VALUE):
                            is_frontier = True
                            break
                    if is_frontier:
                        break
               
                if is_frontier:
                    # Calculer la distance
                    dist = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                    # Ne considérer que les frontières pas trop loin
                    if dist < 50:  # Limiter la distance pour éviter de traverser toute la carte
                        frontier_cells.append((dist, x, y))
           
            if not frontier_cells:
                # S'il n'y a pas de frontières proches, prendre une cellule inexplorée aléatoire
                idx = np.random.randint(0, len(unexplored_cells))
                y, x = unexplored_cells[idx]
            else:
                # Prendre une frontière proche, mais pas la plus proche
                # (pour encourager l'exploration)
                frontier_cells.sort()
                if len(frontier_cells) > 5:
                    # Choisir parmi les 5 frontières les plus proches
                    choice_idx = np.random.randint(0, 5)
                    _, x, y = frontier_cells[choice_idx]
                else:
                    _, x, y = frontier_cells[0]
       
        # Convertir les coordonnées de grille en coordonnées mondiales
        target_x = x * self.map_msg.info.resolution + self.map_msg.info.origin.position.x
        target_y = (self.h - y) * self.map_msg.info.resolution + self.map_msg.info.origin.position.y
       
        return target_x, target_y


def main():
    rclpy.init()

    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()