#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import font
import threading

class RestaurantGUI(Node):
    def __init__(self):
        super().__init__('restaurant_gui')
        
        # Communication ROS
        self.command_pub = self.create_publisher(String, '/gui/command', 10)
        self.status_sub = self.create_subscription(String, '/gui/status', self.update_status, 10)
        
        self.current_status = "WAITING..."

        # --- Configuration de la fenêtre Tkinter ---
        self.root = tk.Tk()
        self.root.title("Commande Robot Restaurant")
        self.root.geometry("600x400")
        self.root.configure(bg="#2c3e50")

        # Layout principal (2 colonnes)
        left_frame = tk.Frame(self.root, bg="#ecf0f1", width=300)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        right_frame = tk.Frame(self.root, bg="#34495e", width=300)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- Colonne Gauche : Boutons Tables ---
        tk.Label(left_frame, text="SÉLECTION TABLE", bg="#ecf0f1", font=("Helvetica", 16, "bold")).pack(pady=10)
        
        btn_container = tk.Frame(left_frame, bg="#ecf0f1")
        btn_container.pack(expand=True)

        # Grille de boutons 4x4
        for row in range(1, 5):
            for num in range(1, 5):
                table_name = f"table_{row}{num}"
                btn = tk.Button(btn_container, text=f"T {row}-{num}", 
                                command=lambda t=table_name: self.send_command(t),
                                width=8, height=2, bg="#3498db", fg="white", font=("Arial", 10, "bold"))
                btn.grid(row=row-1, column=num-1, padx=5, pady=5)

        # --- Colonne Droite : Statut ---
        tk.Label(right_frame, text="ÉTAT DU ROBOT", bg="#34495e", fg="white", font=("Helvetica", 16, "bold")).pack(pady=20)
        
        self.status_label = tk.Label(right_frame, text="IDLE", bg="#e74c3c", fg="white", 
                                     font=("Helvetica", 24, "bold"), width=12, height=2)
        self.status_label.pack(expand=True)

    def send_command(self, table_name):
        """ Envoie l'ordre au robot """
        msg = String()
        msg.data = table_name
        self.command_pub.publish(msg)
        self.get_logger().info(f"Commande envoyée : {table_name}")

    def update_status(self, msg):
        """ Met à jour l'affichage (Appelé par ROS) """
        status = msg.data
        color = "#e74c3c" # Rouge par défaut
        
        if status == "MOVING": color = "#f39c12" # Orange
        elif status == "SERVING": color = "#2ecc71" # Vert
        elif status == "IDLE": color = "#95a5a6" # Gris

        # Mise à jour Thread-safe de l'interface
        self.status_label.config(text=status, bg=color)

    def run(self):
        self.root.mainloop()

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = RestaurantGUI()
    
    # Lancer ROS dans un thread séparé pour ne pas bloquer l'interface
    spin_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
    spin_thread.start()
    
    # Lancer l'interface graphique (Bloquant)
    gui_node.run()
    
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()