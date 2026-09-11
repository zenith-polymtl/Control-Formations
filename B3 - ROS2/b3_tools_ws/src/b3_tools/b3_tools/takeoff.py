#!/usr/bin/env python3
"""
Décollage automatique de la formation B3.3 (node fournie).

Version simplifiée de la node init de nav_stack (aeac-2026) :

1. Demande à l'autopilote les messages de position (set_message_interval).
2. Attend un GPS prêt : une position globale reçue et non nulle.
3. Passe le drone en GUIDED.
4. Arme les moteurs, puis décolle à takeoff_alt (10 m par défaut).
5. Une fois l'altitude atteinte, publie True sur /b3/takeoff/completed et ne
   fait plus rien : le contrôle est libéré pour les autres nodes.

Attention : le drone décolle tout seul dès que le GPS est prêt. Ne lancer la
node que quand on est prêt à voler.

Simplification de simulation seulement : sur un vrai drone, le code ne change
jamais de mode et n'arme jamais les moteurs. Le pilote est toujours responsable
de ces changements ; le code attend qu'il les ait faits, puis agit dans ce cadre.

Si le pilote change de mode (LAND, par exemple) ou si le drone désarme pendant
le décollage, la node abandonne sans insister : le pilote a toujours le dernier mot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode

# Identifiants des messages MAVLink demandés (https://mavlink.io/en/messages/common.html)
LOCAL_POSITION_NED = 32    # alimente /mavros/local_position/pose
GLOBAL_POSITION_INT = 33   # alimente /mavros/global_position/global et rel_alt
REQUESTED_MESSAGES = (LOCAL_POSITION_NED, GLOBAL_POSITION_INT)

COMPLETED_TOPIC = '/b3/takeoff/completed'
ALT_TOLERANCE = 0.5        # m : décollage terminé à takeoff_alt - ALT_TOLERANCE


class TakeoffState:
    WAIT_MESSAGES = "ATTENTE_MESSAGES"
    WAIT_GPS      = "ATTENTE_GPS"
    SET_GUIDED    = "PASSAGE_GUIDED"
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

        self.get_logger().info(f"Node takeoff démarrée : décollage à {self.takeoff_alt} m dès que le GPS est prêt.")

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
        self.declare_parameter('takeoff_alt', 10.0)         # m au-dessus du point de départ
        self.declare_parameter('msg_interval_rate', 10.0)   # Hz demandés pour les messages de position

        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.msg_interval_rate = self.get_parameter('msg_interval_rate').value

    # ------------------------------------------------------------------
    # Services et topics
    # ------------------------------------------------------------------
    def set_up_services(self):
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
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

        self.completed_pub = self.create_publisher(Bool, COMPLETED_TOPIC, latched_qos)

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
            if not self.gps_ok:
                self.get_logger().info("En attente du GPS...", throttle_duration_sec=5.0)
            elif self.mavros_state.armed and self.rel_alt > 1.0:
                self.finish("drone déjà en vol")
            else:
                self._transition(TakeoffState.SET_GUIDED, "GPS prêt")

        elif self._state == TakeoffState.SET_GUIDED:
            if self.in_guided():
                self._transition(TakeoffState.ARMING, "mode GUIDED")
            elif not self.call_pending:
                self.send_set_mode_request()

        elif self._state == TakeoffState.ARMING:
            if not self.in_guided():
                self.abort("le pilote a changé de mode")
            elif not self.call_pending:
                self.send_arm_request()

        elif self._state == TakeoffState.CLIMBING:
            if not self.in_guided():
                self.abort("le pilote a changé de mode")
            elif not self.mavros_state.armed:
                self.abort("drone désarmé pendant la montée")
            elif self.rel_alt >= self.takeoff_alt - ALT_TOLERANCE:
                self.finish(f"altitude atteinte : {self.rel_alt:.1f} m")
            else:
                self.get_logger().info(f"Montée : {self.rel_alt:.1f} / {self.takeoff_alt} m",
                                       throttle_duration_sec=2.0)

    def finish(self, reason: str):
        self._transition(TakeoffState.DONE, reason)
        self.completed_pub.publish(Bool(data=True))
        self.destroy_timer(self.timer)
        self.get_logger().info(f"Décollage terminé, contrôle libéré ({COMPLETED_TOPIC} = True).")

    def abort(self, reason: str):
        self.get_logger().warn(f"Décollage abandonné : {reason}. Relancer la node pour recommencer.")
        self.destroy_timer(self.timer)

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
        if future.result().success:
            self.requests_succeeded += 1
            self.get_logger().info(f"Message {message_id} demandé à {self.msg_interval_rate} Hz")
        else:
            self.get_logger().warn(f"Requête du message {message_id} refusée, nouvel essai.",
                                   throttle_duration_sec=5.0)

        if self.requests_pending == 0 and self.requests_succeeded == len(REQUESTED_MESSAGES):
            self._transition(TakeoffState.WAIT_GPS, "messages de position demandés")

    # ------------------------------------------------------------------
    # Mode, armement et décollage
    # ------------------------------------------------------------------
    def send_set_mode_request(self):
        self.call_pending = True
        future = self.set_mode_client.call_async(SetMode.Request(custom_mode='GUIDED'))
        future.add_done_callback(self.set_mode_callback)

    def set_mode_callback(self, future):
        # Le mode réel se lit dans /mavros/state : step() le vérifie au prochain tic
        self.call_pending = False
        if not future.result().mode_sent:
            self.get_logger().warn("Passage en GUIDED refusé, nouvel essai.", throttle_duration_sec=5.0)

    def send_arm_request(self):
        self.call_pending = True
        future = self.arming_client.call_async(CommandBool.Request(value=True))
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        response = future.result()
        if not response.success:
            # Refus typique : vérifications pré-armement (PreArm) pas encore passées.
            # Le message exact s'affiche dans Mission Planner.
            self.get_logger().warn(f"Armement refusé (result={response.result}), nouvel essai. "
                                   "Voir les messages PreArm dans Mission Planner.",
                                   throttle_duration_sec=5.0)
            self.call_pending = False
            return

        self.get_logger().info("Moteurs armés, envoi du décollage.")
        request = CommandTOL.Request()
        request.altitude = float(self.takeoff_alt)
        request.latitude = self.latitude
        request.longitude = self.longitude
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        self.call_pending = False
        response = future.result()
        if not response.success:
            # On reste en ARMEMENT : le prochain tic réarme et renvoie le décollage.
            self.get_logger().warn(f"Décollage refusé (result={response.result}), nouvel essai.",
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
