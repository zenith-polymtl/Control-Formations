
#import 
import rclpy #bibliotheque de ROS2  fonctions les plus élémentaires init(), create_node()et shutdown()
import time
from rclpy.node import Node #Node classe de ROS2 qui donne acess aux méthode (publishers, subscribers, timers, et paramètres.)
from rclpy.time import Time 
from std_msgs.msg import String, Float32 # messages “simples” standards (entiers, flottants, chaînes, etc.) utilisé dans ce code pour publier sur /arrival.
from geometry_msgs.msg import PoseStamped
from zenmav.core import Zenmav # rend disponible les méthodes comme set_mode, arm, takeoff, local_target,
from zenmav.zenpoint import wp

class suiviDeTrajectoire(Node):
    def __init__(self):
        super().__init__('suiviDeTrajectoire')
        
        #Parametre ROS2 de connexion: 
        self.declare_parameter("zenmav_ip", "tcp:127.0.0.1:5762")
        zenmav_ip = (
            self.get_parameter("zenmav_ip").get_parameter_value().string_value
        )
        
        # Paramètre ROS2 défini
        self.declare_parameter("takeoff_alt", 10.0) 
        self.takeoff_alt = (
            self.get_parameter("takeoff_alt").get_parameter_value().double_value
        )

        self.declare_parameter("look_ahead", 3.0)
        self.look_ahead = (
            self.get_parameter("look_ahead").get_parameter_value().double_value
        )

        
        # Nos publishers et subcribers de notre noeux. 
        
        # signal le fait qu'on est arrivé au premier point
        self.arrival_pub = self.create_publisher(String, '/arrival', 10)
        # C’est le topic par lequel le nœud ROS envoie des consignes de position à MAVROS, 
        # qui les transmet après à ArduPilot pour piloter le drone.
        self.command_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # à chaque nouvelle pose du ballon, le nœud met à jour ses infos
        self.ballon_sub = self.create_subscription(PoseStamped, '/Ballon_pose', self.pos_callback, 10)

        #Stock la position 
        self.last_x = None
        self.last_y = None
        self.last_z = None
        
        #utilie pour le fitlre alpha beta 
        self.x_hat = None
        self.y_hat = None
        self.z_hat = None

        self.vx_hat = 0.0
        self.vy_hat = 0.0
        self.vz_hat = 0.0

        # Gains alpha-beta à ajuster
        self.alpha = 0.6
        self.beta = 0.2

        #dernier timestamp reçu, pour calculer dt
        self.last_stamp = None 
        self.last_stamp_health = None

        self.first = True # first callback flag
        self.follow = True # following flag
        #Initialisation de Zenmav ensuite mouvement 
        self.get_logger().info('Initialized node, sending to target')
        self.drone = Zenmav(zenmav_ip) # Zenmav instance to access high level functions
        self.go_to_first_point() 
        
    def go_to_first_point(self):
        self.drone.set_mode('GUIDED')
        self.drone.arm()
        self.drone.takeoff(self.takeoff_alt)
        self.drone.local_target((10, 20, -50))  # NED

        msg = String()
        msg.data = 'Tommy'
        self.arrival_pub.publish(msg)

    def pos_callback(self, msg: PoseStamped):
        # Lecture de la mesure (position du ballon en ENU)
        self.ballon_stamp = msg.header.stamp
        meas_x = float(msg.pose.position.x)  # E
        meas_y = float(msg.pose.position.y)  # N
        meas_z = float(msg.pose.position.z)  # U

        # Premier échantillon : initialisation
        if self.last_x is None:
            self.last_x = meas_x
            self.last_y = meas_y
            self.last_z = meas_z
            self.last_stamp = self.ballon_stamp
            self.last_stamp_health = self.ballon_stamp

            # Initialisation des états du filtre alpha-beta
            self.x_hat = meas_x
            self.y_hat = meas_y
            self.z_hat = meas_z

            # Démarrage du watchdog
            self.health_timer = self.create_timer(5.0, self.health_check)
            self.first = False

            if self.follow:
                # ENU -> NED : (N, E, D) = (y, x, -z)
                point = wp(meas_y, meas_x, -meas_z, frame='local')
                self.drone.local_target(point, wait_to_reach=False)
                self.get_logger().info(f'FIRST TARGET → {point.coordinates} m NED')

            return

        # Calcul du dt
        dt = (Time.from_msg(self.ballon_stamp) - Time.from_msg(self.last_stamp)).nanoseconds / 1e9
        if dt <= 0:
            # On met juste à jour les anciennes mesures et on quitte
            self.last_stamp = self.ballon_stamp
            self.last_x, self.last_y, self.last_z = meas_x, meas_y, meas_z
            return

        # ======== FILTRE ALPHA-BETA POUR CHAQUE AXE ========

        alpha = self.alpha
        beta = self.beta

        # 1) PRÉDICTION (modèle vitesse constante)
        # position prédite
        x_pred = self.x_hat + self.vx_hat * dt
        y_pred = self.y_hat + self.vy_hat * dt
        z_pred = self.z_hat + self.vz_hat * dt

        # vitesse prédite (constante)
        vx_pred = self.vx_hat
        vy_pred = self.vy_hat
        vz_pred = self.vz_hat

        # 2) RÉSIDUS (erreurs de prédiction)
        rx = meas_x - x_pred
        ry = meas_y - y_pred
        rz = meas_z - z_pred

        # 3) CORRECTION
        self.x_hat = x_pred + alpha * rx
        self.y_hat = y_pred + alpha * ry
        self.z_hat = z_pred + alpha * rz

        self.vx_hat = vx_pred + (beta / dt) * rx
        self.vy_hat = vy_pred + (beta / dt) * ry
        self.vz_hat = vz_pred + (beta / dt) * rz

        # ======== PRÉDICTION À L'HORIZON look_ahead ========

        lookahead = self.look_ahead  # secondes

        px = self.x_hat + self.vx_hat * lookahead
        py = self.y_hat + self.vy_hat * lookahead
        pz = self.z_hat + self.vz_hat * lookahead

        # Optionnel : limiter la distance max pour éviter les commandes déraisonnables
        # (exemple : clamp à 20 m de la position mesurée)
        max_dist = 20.0
        dx = px - meas_x
        dy = py - meas_y
        dz = pz - meas_z
        dist2 = dx*dx + dy*dy + dz*dz
        if dist2 > max_dist * max_dist:
            # On ramène la prédiction plus près du ballon
            scale = max_dist / (dist2 ** 0.5)
            px = meas_x + dx * scale
            py = meas_y + dy * scale
            pz = meas_z + dz * scale

        # Envoi de la commande au drone
        if self.follow:
            cmd = PoseStamped()
            cmd.pose.position.x = px
            cmd.pose.position.y = py
            cmd.pose.position.z = pz

            self.command_pub.publish(cmd)

            self.get_logger().info(
                f'ALPHA-BETA PREDICT {lookahead:.1f}s: '
                f'meas=({meas_x:.2f},{meas_y:.2f},{meas_z:.2f}) ENU, '
                f'est=({self.x_hat:.2f},{self.y_hat:.2f},{self.z_hat:.2f}), '
                f'pred=({px:.2f},{py:.2f},{pz:.2f})'
            )

        # Mise à jour des anciennes mesures pour le dt et le watchdog
        self.last_stamp = self.ballon_stamp
        self.last_x, self.last_y, self.last_z = meas_x, meas_y, meas_z
    def health_check(self):
        # simple watchdog: if stamp didn't advance, stop following, means Balloon pos is stopped.
        # Okay since no need to detect the drop rapidly, not ideal, a bit hacky, works for this, might need something more complex in real life
        if self.last_stamp_health != self.ballon_stamp:
            self.last_stamp_health = self.ballon_stamp
        else:
            self.follow = False
            self.drone.set_mode('RTL')
            
            
def main():
    rclpy.init()
    node = suiviDeTrajectoire()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

