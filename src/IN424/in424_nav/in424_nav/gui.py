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
        
        self.command_pub = self.create_publisher(String, '/gui/command', 10)
        self.status_sub = self.create_subscription(String, '/gui/status', self.update_status, 10)
        
        # Configuration Fenêtre
        self.root = tk.Tk()
        self.root.title("Superviseur Restaurant")
        self.root.geometry("1000x600") 
        self.root.configure(bg="#2c3e50")

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # === GAUCHE (SELECTION) ===
        left_frame = tk.Frame(self.root, bg="#ecf0f1")
        left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        tk.Label(left_frame, text="CARTE DE LA SALLE", bg="#ecf0f1", 
                 font=("Helvetica", 18, "bold"), fg="#2c3e50").pack(pady=20)
        
        btn_container = tk.Frame(left_frame, bg="#ecf0f1")
        btn_container.pack(expand=True)

        headers = ["Rangée 1\n(Gauche)", "Rangée 2\n(Centre G)", "Rangée 3\n(Centre D)", "Rangée 4\n(Droite)"]
        for col_idx, title in enumerate(headers):
            tk.Label(btn_container, text=title, bg="#ecf0f1", fg="#7f8c8d", font=("Arial", 9, "italic")).grid(row=0, column=col_idx, padx=10, pady=(0, 15))

        for r in range(1, 5): 
            for t in range(1, 5):
                table_name = f"table_{r}{t}"
                btn_bg = "#27ae60" if r in [1, 2] else "#2980b9"
                btn = tk.Button(btn_container, text=f"Table {r}-{t}", 
                                command=lambda x=table_name: self.send_command(x),
                                width=10, height=2, bg=btn_bg, fg="white", 
                                font=("Arial", 10, "bold"), activebackground="#bdc3c7")
                btn.grid(row=t, column=r-1, padx=8, pady=8)

        # === DROITE (STATUT & RETOUR) ===
        right_frame = tk.Frame(self.root, bg="#34495e")
        right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        tk.Label(right_frame, text="ÉTAT DU ROBOT", bg="#34495e", fg="#ecf0f1", 
                 font=("Helvetica", 18, "bold")).pack(pady=40)
        
        self.status_label = tk.Label(right_frame, text="IDLE", bg="#95a5a6", fg="white", 
                                     font=("Helvetica", 30, "bold"), width=12, height=2)
        self.status_label.pack(expand=True)

        # --- NOUVEAU BOUTON DE RETOUR ---
        self.return_btn = tk.Button(right_frame, text="PLAT RÉCUPÉRÉ\n(Retour Base)", 
                                    command=self.send_return,
                                    state=tk.DISABLED, # Désactivé par défaut
                                    bg="#7f8c8d", fg="white",
                                    font=("Helvetica", 16, "bold"), 
                                    width=20, height=3, bd=0)
        self.return_btn.pack(pady=50)
        
        self.log_label = tk.Label(right_frame, text="En attente...", bg="#34495e", fg="#bdc3c7", font=("Courier", 10))
        self.log_label.pack(side=tk.BOTTOM, pady=20)

    def send_command(self, table_name):
        msg = String(); msg.data = table_name
        self.command_pub.publish(msg)
        self.get_logger().info(f"Ordre envoyé : {table_name}")
        self.log_label.config(text=f"Destination : {table_name}")

    def send_return(self):
        """ Envoie l'ordre de retour quand on clique sur le gros bouton """
        msg = String(); msg.data = "RETURN"
        self.command_pub.publish(msg)
        self.get_logger().info("Ordre de retour envoyé !")
        self.return_btn.config(state=tk.DISABLED, bg="#7f8c8d", text="RETOUR EN COURS...")

    def update_status(self, msg):
        status = msg.data
        color = "#95a5a6"
        
        # Logique des couleurs
        if status == "MOVING": color = "#f39c12"
        elif status == "SERVING": color = "#2ecc71"
        elif status == "IDLE": color = "#95a5a6"

        self.root.after(0, lambda: self._safe_update(status, color))

    def _safe_update(self, status, color):
        self.status_label.config(text=status, bg=color)
        
        # --- GESTION INTELLIGENTE DU BOUTON RETOUR ---
        if status == "SERVING":
            # Le robot attend -> On active le bouton en VERT FLUO
            self.return_btn.config(state=tk.NORMAL, bg="#e74c3c", text="CONFIRMER\nRÉCUPÉRATION")
        elif status == "MOVING":
            self.return_btn.config(state=tk.DISABLED, bg="#f39c12", text="EN DÉPLACEMENT...")
        else:
            self.return_btn.config(state=tk.DISABLED, bg="#7f8c8d", text="EN ATTENTE")

    def run(self): self.root.mainloop()

def ros_spin_thread(node): rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = RestaurantGUI()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
    spin_thread.start()
    gui_node.run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()