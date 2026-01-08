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

        # --- Configuration de la fenêtre ---
        self.root = tk.Tk()
        self.root.title("Superviseur Restaurant")
        self.root.geometry("1000x600") 
        self.root.configure(bg="#2c3e50")

        # Grille principale (50% Gauche / 50% Droite)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # === CADRE GAUCHE : SÉLECTION DES TABLES ===
        left_frame = tk.Frame(self.root, bg="#ecf0f1")
        left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        tk.Label(left_frame, text="CARTE DE LA SALLE", bg="#ecf0f1", 
                 font=("Helvetica", 18, "bold"), fg="#2c3e50").pack(pady=20)
        
        # Conteneur pour la grille de boutons
        btn_container = tk.Frame(left_frame, bg="#ecf0f1")
        btn_container.pack(expand=True)

        # --- Génération de la Grille "Colonnes = Rangées" ---
        
        # En-têtes des colonnes
        headers = ["Rangée 1\n(Gauche)", "Rangée 2\n(Centre G)", "Rangée 3\n(Centre D)", "Rangée 4\n(Droite)"]
        for col_idx, title in enumerate(headers):
            lbl = tk.Label(btn_container, text=title, bg="#ecf0f1", fg="#7f8c8d", font=("Arial", 9, "italic"))
            lbl.grid(row=0, column=col_idx, padx=10, pady=(0, 15))

        # Création des boutons
        # r = Rangée du restaurant (1..4) -> Devient la COLONNE de l'interface
        # t = Numéro de table (1..4)      -> Devient la LIGNE de l'interface
        for r in range(1, 5): 
            for t in range(1, 5):
                table_name = f"table_{r}{t}"
                
                # Couleur différente pour distinguer les zones (Vert/Bleu)
                btn_bg = "#27ae60" if r in [1, 2] else "#2980b9" # Vert pour gauche, Bleu pour droite
                
                btn = tk.Button(btn_container, text=f"Table {r}-{t}", 
                                command=lambda x=table_name: self.send_command(x),
                                width=10, height=2,  # Taille réduite
                                bg=btn_bg, fg="white", 
                                font=("Arial", 10, "bold"),
                                activebackground="#bdc3c7")
                
                # Grid : row=t (décalé de 1 pour le header), column=r-1
                btn.grid(row=t, column=r-1, padx=8, pady=8)


        # === CADRE DROIT : STATUT DU ROBOT ===
        right_frame = tk.Frame(self.root, bg="#34495e")
        right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        tk.Label(right_frame, text="ÉTAT DU ROBOT", bg="#34495e", fg="#ecf0f1", 
                 font=("Helvetica", 18, "bold")).pack(pady=40)
        
        self.status_label = tk.Label(right_frame, text="IDLE", 
                                     bg="#95a5a6", fg="white", 
                                     font=("Helvetica", 30, "bold"), 
                                     width=12, height=2)
        self.status_label.pack(expand=True)
        
        # Petit log visuel en dessous
        self.log_label = tk.Label(right_frame, text="En attente...", bg="#34495e", fg="#bdc3c7", font=("Courier", 10))
        self.log_label.pack(side=tk.BOTTOM, pady=20)

    def send_command(self, table_name):
        msg = String()
        msg.data = table_name
        self.command_pub.publish(msg)
        self.get_logger().info(f"Ordre envoyé : {table_name}")
        self.log_label.config(text=f"Dernier ordre : {table_name}")

    def update_status(self, msg):
        status = msg.data
        color = "#95a5a6" # Gris
        
        if status == "MOVING": color = "#f39c12" # Orange
        elif status == "SERVING": color = "#2ecc71" # Vert
        elif status == "IDLE": color = "#95a5a6" # Gris

        # Thread-safe update
        self.root.after(0, lambda: self._safe_update(status, color))

    def _safe_update(self, text, color):
        self.status_label.config(text=text, bg=color)

    def run(self):
        self.root.mainloop()

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = RestaurantGUI()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
    spin_thread.start()
    gui_node.run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()