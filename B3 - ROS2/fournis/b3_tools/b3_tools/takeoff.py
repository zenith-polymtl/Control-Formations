#!/usr/bin/env python3
"""
Décollage automatique de la formation B3.3 (node fournie).

Version simplifiée de la node init de nav_stack (aeac-2026) :

1. Demande à l'autopilote les messages de position (set_message_interval).
2. Attend un GPS prêt : une position globale reçue et non nulle.
3. Attend que le pilote passe le drone en GUIDED (Mission Planner ou
   ros2 service call /mavros/set_mode). La node ne change jamais le mode
   elle-même : c'est le pilote qui donne le feu vert.
4. Arme les moteurs, puis décolle à takeoff_alt.
5. Une fois l'altitude atteinte, publie True sur /b3/takeoff/completed et ne
   fait plus rien : le contrôle est libéré pour les autres nodes.

Si le pilote quitte GUIDED en cours de route (LAND, par exemple), la node
revient à l'attente de GUIDED sans insister.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval

# Identifiants des messages MAVLink demandés (https://mavlink.io/en/messages/common.html)
LOCAL_POSITION_NED = 32    # alimente /mavros/local_position/pose
GLOBAL_POSITION_INT = 33   # alimente /mavros/global_position/global et rel_alt
REQUESTED_MESSAGES = (LOCAL_POSITION_NED, GLOBAL_POSITION_INT)


class TakeoffState:
    WAIT_MESSAGES = "ATTENTE_MESSAGES"
    WAIT_GPS      = "ATTENTE_GPS"
    WAIT_GUIDED   = "ATTENTE_GUIDED"
    ARMING        = "ARMEMENT"
    CLIMBING      = "MONTEE"
    DONE          = "TERMINE"


class Takeoff(Node):
    def __init__(self):
        super().__init__('takeoff')

        self.define_initial_state()
        self.set_up_parameters()
        self.set_up_services()
        self.set_up_topics()

        # La machine à états avance au rythme de ce timer (une tentative par seconde au plus)
        self.timer = self.create_timer(1.0, self.step)

        self.get_logger().info(f"Node takeoff démarrée : décollage à {self.takeoff_alt} m une fois en GUIDED.")

    # ------------------------------------------------------------------
    # État
    # ------------------------------------------------------------------
    def define_initial_state(self):
        self._state = TakeoffState.WAIT_MESSAGES
        self.mavros_state = None
        self.gps_ok = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.rel_alt = 0.0
        self.requests_pending = 0
        self.requests_succeeded = 0
        self.call_pending = False

    def _transition(self, new_state: str, reason: str = ""):
        """Toutes les transitions passent par ici, pour les voir dans les logs."""
        if new_state == self._state:
            return
        self.get_logger().info(f"[ETAT] {self._state} → {new_state}" + (f"  ({reason})" if reason else ""))
        self._state = new_state

    # ------------------------------------------------------------------
    # Paramètres
    # ------------------------------------------------------------------
    def set_up_parameters(self):
        self.declare_parameter('takeoff_alt', 5.0)          # m au-dessus du point de départ
        self.declare_parameter('alt_tolerance', 0.5)        # m : décollage terminé à takeoff_alt - alt_tolerance
        self.declare_parameter('msg_interval_rate', 10.0)   # Hz demandés pour les messages de position
        self.declare_parameter('completed_topic', '/b3/takeoff/completed')

        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.alt_tolerance = self.get_parameter('alt_tolerance').value
        self.msg_interval_rate = self.get_parameter('msg_interval_rate').value
        self.completed_topic = self.get_parameter('completed_topic').value

    # ------------------------------------------------------------------
    # Services et topics
    # ------------------------------------------------------------------
    def set_up_services(self):
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    def set_up_topics(self):
        # Les topics de capteurs de mavros sont publiés en BEST_EFFORT : un
        # subscriber RELIABLE ne recevrait rien (seulement un avertissement de
        # QoS incompatible, facile à rater).
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # TRANSIENT_LOCAL : le message de fin reste disponible pour les nodes
        # qui démarrent plus tard (la téléop, par exemple).
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_be)
        self.rel_alt_sub = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_be)

        self.completed_pub = self.create_publisher(Bool, self.completed_topic, latched_qos)

    def state_callback(self, msg: State):
        self.mavros_state = msg

    def gps_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        # Même test que nav_stack/init : une position reçue et non nulle veut
        # dire que l'autopilote a une position globale valide.
        self.gps_ok = msg.latitude != 0.0 and msg.longitude != 0.0

    def rel_alt_callback(self, msg: Float64):
        self.rel_alt = msg.data

    # ------------------------------------------------------------------
    # Machine à états
    # ------------------------------------------------------------------
    def in_guided(self):
        return self.mavros_state is not None and self.mavros_state.mode == 'GUIDED'

    def step(self):
        if self._state == TakeoffState.WAIT_MESSAGES:
            self.request_message_intervals()

        elif self._state == TakeoffState.WAIT_GPS:
            if self.gps_ok:
                self._transition(TakeoffState.WAIT_GUIDED, "GPS prêt")
                self.get_logger().info(
                    "Passer le drone en GUIDED pour décoller (Mission Planner : Actions, Set Mode, Guided).")
            else:
                self.get_logger().info("En attente du GPS...", throttle_duration_sec=5.0)

        elif self._state == TakeoffState.WAIT_GUIDED:
            if not self.in_guided():
                return
            if self.mavros_state.armed and self.rel_alt > 1.0:
                self.finish("drone déjà en vol")
            else:
                self._transition(TakeoffState.ARMING, "mode GUIDED")

        elif self._state == TakeoffState.ARMING:
            if not self.in_guided():
                self._transition(TakeoffState.WAIT_GUIDED, "le pilote a quitté GUIDED")
            elif not self.call_pending:
                self.send_arm_request()

        elif self._state == TakeoffState.CLIMBING:
            if not self.in_guided():
                self._transition(TakeoffState.WAIT_GUIDED, "le pilote a quitté GUIDED")
            elif not self.mavros_state.armed:
                # Montée interrompue (une consigne de mouvement reçue trop tôt,
                # par exemple) : le drone a désarmé au sol, on recommence.
                self._transition(TakeoffState.WAIT_GUIDED, "drone désarmé pendant la montée")
            elif self.rel_alt >= self.takeoff_alt - self.alt_tolerance:
                self.finish(f"altitude atteinte : {self.rel_alt:.1f} m")
            else:
                self.get_logger().info(f"Montée : {self.rel_alt:.1f} / {self.takeoff_alt} m",
                                       throttle_duration_sec=2.0)

    def finish(self, reason: str):
        self._transition(TakeoffState.DONE, reason)
        self.completed_pub.publish(Bool(data=True))
        self.destroy_timer(self.timer)
        self.get_logger().info(f"Décollage terminé, contrôle libéré ({self.completed_topic} = True).")

    # ------------------------------------------------------------------
    # Requête des messages de position
    # ------------------------------------------------------------------
    def request_message_intervals(self):
        if self.requests_pending > 0:
            return
        # Avant la connexion à l'autopilote, mavros répondrait « succès » sans
        # rien envoyer : on attend donc connected dans /mavros/state.
        if self.mavros_state is None or not self.mavros_state.connected:
            self.get_logger().info("En attente de la connexion de mavros à l'autopilote "
                                   "(simulation démarrée?)", throttle_duration_sec=5.0)
            return
        if not self.msg_interval_client.service_is_ready():
            return

        self.requests_succeeded = 0
        for message_id in REQUESTED_MESSAGES:
            request = MessageInterval.Request()
            request.message_id = message_id
            request.message_rate = self.msg_interval_rate
            future = self.msg_interval_client.call_async(request)
            future.add_done_callback(lambda f, mid=message_id: self.message_interval_callback(f, mid))
            self.requests_pending += 1

    def message_interval_callback(self, future, message_id):
        self.requests_pending -= 1
        try:
            success = future.result().success
        except Exception as e:
            self.get_logger().error(f"Appel de set_message_interval échoué : {e}")
            success = False

        if success:
            self.requests_succeeded += 1
            self.get_logger().info(f"Message {message_id} demandé à {self.msg_interval_rate} Hz")
        else:
            self.get_logger().warn(f"Requête du message {message_id} refusée, nouvel essai.",
                                   throttle_duration_sec=5.0)

        if self.requests_pending == 0 and self.requests_succeeded == len(REQUESTED_MESSAGES):
            self._transition(TakeoffState.WAIT_GPS, "messages de position demandés")

    # ------------------------------------------------------------------
    # Armement et décollage
    # ------------------------------------------------------------------
    def send_arm_request(self):
        if not self.arming_client.service_is_ready():
            return
        self.call_pending = True
        future = self.arming_client.call_async(CommandBool.Request(value=True))
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Appel d'armement échoué : {e}")
            self.call_pending = False
            return

        if not response.success:
            # Refus typique : vérifications pré-armement (PreArm) pas encore passées.
            # Le message exact s'affiche dans Mission Planner.
            self.get_logger().warn(f"Armement refusé (result={response.result}), nouvel essai. "
                                   "Voir les messages PreArm dans Mission Planner.",
                                   throttle_duration_sec=5.0)
            self.call_pending = False
            return

        if self._state != TakeoffState.ARMING:
            # Le pilote a quitté GUIDED pendant l'appel : pas de décollage
            self.call_pending = False
            return

        self.get_logger().info("Moteurs armés, envoi du décollage.")
        self.send_takeoff_request()

    def send_takeoff_request(self):
        request = CommandTOL.Request()
        request.altitude = float(self.takeoff_alt)
        request.latitude = self.latitude
        request.longitude = self.longitude
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        self.call_pending = False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Appel de décollage échoué : {e}")
            return

        if not response.success:
            # On reste en ARMEMENT : le prochain tic réarme et renvoie le décollage.
            self.get_logger().error(f"Décollage refusé (result={response.result}). Nouvel essai.",
                                    throttle_duration_sec=5.0)
        elif self._state == TakeoffState.ARMING:
            self._transition(TakeoffState.CLIMBING, "décollage accepté")


def main(args=None):
    rclpy.init(args=args)
    node = Takeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
