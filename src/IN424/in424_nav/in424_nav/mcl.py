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