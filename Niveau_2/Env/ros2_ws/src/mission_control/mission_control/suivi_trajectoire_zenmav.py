#!/usr/bin/env python3
# ========== DÉBUT TÂCHE 1 : En-tête & imports ==========
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from zenmav.core import Zenmav
from zenmav.zenpoint import wp
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu  # import conservé au besoin
import tf_transformations         # import conservé au besoin

# ========== FIN TÂCHE 1 ==========


class FollowerZenmav(Node):
    def __init__(self):
        super().__init__('follower_zenmav')

        # ===== DÉBUT TÂCHE 3 : Paramètres connexion/mission =====
        self.declare_parameter("zenmav_ip", "tcp:127.0.0.1:5762")
        self.declare_parameter("takeoff_alt", 10.0)
        self.declare_parameter("look_ahead", 2.0)               # s
        self.declare_parameter("autopublish_arrival", True)
        self.declare_parameter("arrival_name", "MonNom")
        self.declare_parameter("limit_altitude_delta", 0.0)     # 0=off
        # ===== FIN TÂCHE 3 =====

        # ========== DÉBUT TÂCHE 4 : Paramètres de contrôle ==========
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("kx", 0.8)
        self.declare_parameter("ky", 0.8)
        self.declare_parameter("kz", 0.8)
        self.declare_parameter("vmax_xy", 5.0)    # m/s
        self.declare_parameter("vmax_z", 2.0)     # m/s
        self.declare_parameter("amax_xy", 3.0)    # m/s^2
        self.declare_parameter("amax_z", 1.0)     # m/s^2
        # ========== FIN TÂCHE 4 ==========

        # ========== DÉBUT TÂCHE 5 : Paramètres ROS (topics) ==========
        self.declare_parameter("balloon_topic", "/Ballon_pose")
        self.declare_parameter("drone_pose_topic", "/mavros/local_position/pose")
        # ========== FIN TÂCHE 5 ==========

        # ========== DÉBUT TÂCHE 6 : Lecture/stockage des paramètres ==========
        self.zenmav_ip    = self.get_parameter("zenmav_ip").value
        self.takeoff_alt  = float(self.get_parameter("takeoff_alt").value)
        self.look_ahead   = float(self.get_parameter("look_ahead").value)
        self.auto_arrival = bool(self.get_parameter("autopublish_arrival").value)
        self.arrival_name = str(self.get_parameter("arrival_name").value)
        self.alt_delta    = float(self.get_parameter("limit_altitude_delta").value)

        self.rate_hz   = float(self.get_parameter("rate_hz").value)
        self.kx        = float(self.get_parameter("kx").value)
        self.ky        = float(self.get_parameter("ky").value)
        self.kz        = float(self.get_parameter("kz").value)
        self.vmax_xy   = float(self.get_parameter("vmax_xy").value)
        self.vmax_z    = float(self.get_parameter("vmax_z").value)
        self.amax_xy   = float(self.get_parameter("amax_xy").value)
        self.amax_z    = float(self.get_parameter("amax_z").value)

        self.balloon_topic    = str(self.get_parameter("balloon_topic").value)
        self.drone_pose_topic = str(self.get_parameter("drone_pose_topic").value)
        # ========== FIN TÂCHE 6 ==========

        # ========== DÉBUT TÂCHE 7 : I/O ROS ==========
        self.arrival_pub = self.create_publisher(String, "/arrival", 10)
        self.stats_pub   = self.create_publisher(String, "/follow_stats", 10)

        self.sub_balloon = self.create_subscription(
            PoseStamped, self.balloon_topic, self.on_balloon, 10
        )

        # IMPORTANT : MAVROS /local_position/pose est souvent publié en BEST_EFFORT.
        # On s'aligne avec le profil capteur pour éviter l’incompatibilité de fiabilité.
        self.sub_drone = self.create_subscription(
            PoseStamped,
            self.drone_pose_topic,
            self.on_drone_pose,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        self.sub_monitor = self.create_subscription(
            String, "/monitor", self.on_monitor, 10
        )
        # ========== FIN TÂCHE 7 ==========

        # ========== DÉBUT TÂCHE 8 : État interne ==========
        # Suivi désactivé par défaut : attend un GO
        self.follow = False

        # Ballon (ENU)
        self.ball_seen  = False
        self.ball_xyz   = None        # (x,y,z)
        self.ball_v     = (0.0, 0.0, 0.0)
        self.ball_stamp = None
        self._arrival_timer = None
        self._arrival_sent  = 0

        # Drone (ENU)
        self.drone_xyz   = None       # (x,y,z)
        self.drone_stamp = None

        # Garde d’altitude : n’autorise le suivi qu’une fois ~10 m atteints
        self._alt_gate_done = False
        self._alt_gate_margin = 0.5   # marge 0.5 m autour de takeoff_alt

        # Commande (mémoires pour limiter accélération)
        self.last_cmd_v_xy = (0.0, 0.0)  # (vx,vy)
        self.last_cmd_v_z  = 0.0
        # ========== FIN TÂCHE 8 ==========

        # ========== DÉBUT TÂCHE 9 : Timers (contrôle, métriques, watchdog) ==========
        self._last_tick = None
        self._tick_timer    = self.create_timer(1.0 / self.rate_hz, self.control_tick)
        self._metrics_timer = self.create_timer(1.0, self.metrics_tick)
        self.create_timer(5.0, self.health_check)
        # ========== FIN TÂCHE 9 ==========

        # ========== DÉBUT TÂCHE 10 : Connexion ZenMav & décollage ==========
        self.get_logger().info(f"Connexion ZenMav à {self.zenmav_ip}…")
        self.drone = Zenmav(self.zenmav_ip)
        threading.Thread(target=self._arm_and_takeoff_thread, daemon=True).start()
        # ========== FIN TÂCHE 10 ==========

        # ========== DÉBUT TÂCHE 11 : Envoi automatique d’/arrival (optionnel) ==========
        if self.auto_arrival:
            self._arrival_timer = self.create_timer(0.5, self._repeat_arrival)
        self.get_logger().info("Follower ZenMav prêt (en attente de GO).")
        # ========== FIN TÂCHE 11 ==========

    # ========== FIN TÂCHE 2 ==========

    # ========== DÉBUT TÂCHE 12 : Callback cible (/Ballon_pose) ==========
    def on_balloon(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        stamp = msg.header.stamp

        # Vitesse par différences finies
        if self.ball_stamp is not None and self.ball_xyz is not None:
            dt = (Time.from_msg(stamp) - Time.from_msg(self.ball_stamp)).nanoseconds * 1e-9
            if dt > 0.0:
                vx = (x - self.ball_xyz[0]) / dt
                vy = (y - self.ball_xyz[1]) / dt
                vz = (z - self.ball_xyz[2]) / dt
                self.ball_v = (vx, vy, vz)

        self.ball_xyz = (x, y, z)
        self.ball_stamp = stamp

        if not self.ball_seen:
            self.ball_seen = True
            self._stop_arrival_timer()
            self.get_logger().info("Première pose du ballon reçue.")
    # ========== FIN TÂCHE 12 ==========

    # ========== DÉBUT TÂCHE 13 : Callback pose drone ==========
    def on_drone_pose(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        self.drone_xyz = (x, y, z)
        self.drone_stamp = msg.header.stamp

        # Débloque la garde d’altitude quand ~10 m atteints
        if not self._alt_gate_done and z >= (self.takeoff_alt - self._alt_gate_margin):
            self._alt_gate_done = True
            self.get_logger().info("Altitude atteinte → suivi autorisé.")
    # ========== FIN TÂCHE 13 ==========

    # ========== DÉBUT TÂCHE 14 : Callback /monitor (GO/STOP) ==========
    def on_monitor(self, msg: String):
        data = (msg.data or "").strip().upper()
        if data in ("GO", "STOP"):
            self._stop_arrival_timer()

        if data == "GO":
            try:
                self.drone.set_mode("GUIDED")
            except Exception as e:
                self.get_logger().warn(f"GUIDED set failed: {e}")
            self.follow = True
            self.get_logger().info("GO reçu → suivi ACTIVÉ (GUIDED).")

        elif data == "STOP":
            self.follow = False
            try:
                self.drone.set_mode("BRAKE")
            except Exception:
                self.drone.set_mode("LOITER")
            self.get_logger().info("STOP reçu → suivi DÉSACTIVÉ.")
    # ========== FIN TÂCHE 14 ==========

    # ========== DÉBUT TÂCHE 15 : Boucle de contrôle ==========
    def control_tick(self):
        # dt pour limitation d'accélération
        now = self.get_clock().now()
        if self._last_tick is None:
            self._last_tick = now
            return
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt <= 0.0:
            return

        # Conditions nécessaires
        if not self.follow:
            return
        if not self.ball_seen or self.ball_xyz is None or self.drone_xyz is None:
            return

        # Garde d’altitude : n’avance pas vers la cible avant ~10 m
        if not self._alt_gate_done:
            # on laisse MAVROS/monitor recevoir les poses pendant la montée
            return

        # 1) Prédiction de la cible (lookahead, ENU)
        L = self.look_ahead
        bx, by, bz = self.ball_xyz
        bvx, bvy, bvz = self.ball_v
        px = bx + bvx * L
        py = by + bvy * L
        pz = bz + bvz * L

        # 2) Erreur drone→cible prédite (ENU)
        dx = px - self.drone_xyz[0]
        dy = py - self.drone_xyz[1]
        dz = pz - self.drone_xyz[2]

        # 3) Loi P → vitesse désirée (avant limites)
        vdes_x = self.kx * dx
        vdes_y = self.ky * dy
        vdes_z = self.kz * dz

        # 4) Limitation de vitesse
        vxy_mag = math.hypot(vdes_x, vdes_y)
        if vxy_mag > self.vmax_xy > 0.0:
            scale = self.vmax_xy / vxy_mag
            vdes_x *= scale
            vdes_y *= scale
        if abs(vdes_z) > self.vmax_z > 0.0:
            vdes_z = math.copysign(self.vmax_z, vdes_z)

        # 5) Limitation d'accélération (rampe entre last_cmd_v et vdes)
        last_vx, last_vy = self.last_cmd_v_xy
        last_vz = self.last_cmd_v_z

        # XY vectoriel
        dvx = vdes_x - last_vx
        dvy = vdes_y - last_vy
        dvmag = math.hypot(dvx, dvy)
        amax_xy_dt = self.amax_xy * dt
        if dvmag > amax_xy_dt > 0.0:
            scale = amax_xy_dt / dvmag
            dvx *= scale
            dvy *= scale
        cmd_vx = last_vx + dvx
        cmd_vy = last_vy + dvy

        # Z scalaire
        dvz = vdes_z - last_vz
        amax_z_dt = self.amax_z * dt
        if abs(dvz) > amax_z_dt > 0.0:
            dvz = math.copysign(amax_z_dt, dvz)
        cmd_vz = last_vz + dvz

        self.last_cmd_v_xy = (cmd_vx, cmd_vy)
        self.last_cmd_v_z  = cmd_vz

        # 6) Intégration courte → cible position suivante (ENU)
        tx = self.drone_xyz[0] + cmd_vx * dt
        ty = self.drone_xyz[1] + cmd_vy * dt
        tz = self.drone_xyz[2] + cmd_vz * dt

        # Option : clamp altitude autour de takeoff_alt
        if self.alt_delta > 0.0:
            tz = max(self.takeoff_alt - self.alt_delta,
                     min(self.takeoff_alt + self.alt_delta, tz))

        # 7) ENU → NED et envoi waypoint local
        ned = (ty, tx, -tz)  # (N,E,D)
        try:
            self.drone.local_target(wp(*ned, frame="local"))
        except Exception as e:
            self.get_logger().warn(f"local_target échec: {e}")
    # ========== FIN TÂCHE 15 ==========

    # ========== DÉBUT TÂCHE 16 : Instrumentation ==========
    def metrics_tick(self):
        if not (self.ball_xyz and self.drone_xyz):
            return
        bx, by, bz = self.ball_xyz
        dx = bx - self.drone_xyz[0]
        dy = by - self.drone_xyz[1]
        dz = bz - self.drone_xyz[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        stats = (
            f"dist={dist:.2f} m | e=({dx:.2f},{dy:.2f},{dz:.2f}) m | "
            f"vcmd=({self.last_cmd_v_xy[0]:.2f},{self.last_cmd_v_xy[1]:.2f},{self.last_cmd_v_z:.2f}) m/s"
        )
        self.stats_pub.publish(String(data=stats))
        self.get_logger().info(stats)
    # ========== FIN TÂCHE 16 ==========

    # ========== DÉBUT TÂCHE 17 : Watchdog santé ==========
    def health_check(self):
        if self.ball_stamp is None:
            return
        if not hasattr(self, "_last_health_stamp"):
            self._last_health_stamp = self.ball_stamp
            return
        if self.ball_stamp == self._last_health_stamp:
            self.follow = False
            try:
                self.drone.set_mode("RTL")
                self.get_logger().warn("Ballon immobile → RTL.")
            except Exception:
                self.get_logger().warn("Ballon immobile → suivi arrêté.")
        else:
            self._last_health_stamp = self.ball_stamp
    # ========== FIN TÂCHE 17 ==========

    # ========== DÉBUT TÂCHE 18 : Thread armement/décollage ==========
    def _arm_and_takeoff_thread(self):
        try:
            self.get_logger().info("Mode GUIDED, armement, décollage…")
            self.drone.set_mode("GUIDED")
            self.drone.arm()
            self.drone.takeoff(self.takeoff_alt)
            self.get_logger().info("Commande de décollage envoyée.")
        except Exception as e:
            self.get_logger().error(f"Échec arm/décollage ZenMav : {e}")
    # ========== FIN TÂCHE 18 ==========

    # ========== DÉBUT TÂCHE 19 : Utilitaires /arrival ==========
    def _repeat_arrival(self):
        if self.ball_seen or self._arrival_sent >= 16:
            self._stop_arrival_timer()
            return
        self.arrival_pub.publish(String(data=self.arrival_name))
        self._arrival_sent += 1
        self.get_logger().info(f"Arrival envoyé ({self._arrival_sent})")

    def _stop_arrival_timer(self):
        if self._arrival_timer is not None:
            self.destroy_timer(self._arrival_timer)
            self._arrival_timer = None
    # ========== FIN TÂCHE 19 ==========


# ========== DÉBUT TÂCHE 20 : Point d'entrée ==========
def main():
    rclpy.init()
    node = FollowerZenmav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# ========== FIN TÂCHE 20 ==========
