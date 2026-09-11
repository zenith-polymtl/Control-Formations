#!/usr/bin/env python3
"""
Décollage automatique de la formation B3 (node fournie, outil de confort).

Enchaîne, une étape par seconde, la même suite que les trois « ros2 service
call » de B3.3 section 2.5 :

1. attendre que mavros soit connecté à l'autopilote ;
2. demander les messages de position 32 et 33 à 10 Hz (B3.3 section 3) ;
3. passer en GUIDED, renvoyé chaque seconde tant que le mode ne change pas ;
4. armer, renvoyé chaque seconde tant que le drone n'est pas armé (ArduPilot
   refuse tant que ses vérifications pré-armement, GPS compris, ne passent pas) ;
5. décoller à takeoff_alt, puis afficher l'altitude jusqu'à ce qu'elle soit atteinte.

    ros2 run b3_tools takeoff
    ros2 run b3_tools takeoff --ros-args -p takeoff_alt:=20.0

Le drone décolle tout seul dès que l'autopilote accepte : ne lancer la node que
quand on est prêt à voler.

Simulation seulement. Sur un vrai drone et dans le code final de compétition,
le code ne change jamais de mode et n'arme jamais les moteurs : le pilote en
est responsable, le code attend qu'il l'ait fait et agit dans ce cadre.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode

LOCAL_POSITION_NED = 32    # alimente /mavros/local_position/pose
GLOBAL_POSITION_INT = 33   # alimente /mavros/global_position/rel_alt


class Takeoff(Node):
    def __init__(self):
        super().__init__('takeoff')

        self.declare_parameter('takeoff_alt', 10.0)    # m au-dessus du point de départ
        self.takeoff_alt = self.get_parameter('takeoff_alt').value

        self.state = None       # dernier /mavros/state reçu
        self.rel_alt = 0.0      # altitude au-dessus du point de départ
        self.step = 'messages'  # étape en cours
        self.waiting = False    # une requête est partie, on attend sa réponse

        # Les topics de capteurs de mavros sont publiés en BEST_EFFORT (B3.3 section 2.4)
        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_be)

        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.timer = self.create_timer(1.0, self.tick)   # une étape par seconde au plus
        self.get_logger().info(f"Décollage à {self.takeoff_alt} m dès que l'autopilote accepte.")

    def state_callback(self, msg):
        self.state = msg

    def rel_alt_callback(self, msg):
        self.rel_alt = msg.data

    def call(self, client, request, on_response):
        """Envoie une requête sans bloquer ; on_response reçoit la réponse."""
        if self.waiting or not client.service_is_ready():
            return
        self.waiting = True
        future = client.call_async(request)
        future.add_done_callback(lambda future: self.done(future, on_response))

    def done(self, future, on_response):
        self.waiting = False
        on_response(future.result())

    def tick(self):
        if self.state is None or not self.state.connected:
            self.get_logger().info("En attente de mavros (simulation démarrée ?)", throttle_duration_sec=5.0)
            return

        if self.step == 'messages':
            for message_id in (LOCAL_POSITION_NED, GLOBAL_POSITION_INT):
                self.msg_interval_client.call_async(
                    MessageInterval.Request(message_id=message_id, message_rate=10.0))
            self.get_logger().info("Messages de position demandés à 10 Hz.")
            self.step = 'guided'

        elif self.step == 'guided':
            if self.state.mode == 'GUIDED':
                self.step = 'armement'
            else:   # le résultat se lit dans /mavros/state au tic suivant
                self.call(self.set_mode_client, SetMode.Request(custom_mode='GUIDED'), lambda r: None)

        elif self.step == 'armement':
            if self.state.armed:
                self.step = 'decollage'
            else:
                self.call(self.arming_client, CommandBool.Request(value=True), self.arm_response)

        elif self.step == 'decollage':
            self.call(self.takeoff_client, CommandTOL.Request(altitude=float(self.takeoff_alt)),
                      self.takeoff_response)

        elif self.step == 'montee':
            if self.rel_alt >= self.takeoff_alt - 0.5:
                self.get_logger().info(f"Décollage terminé : {self.rel_alt:.1f} m.")
                self.destroy_timer(self.timer)
            else:
                self.get_logger().info(f"Montée : {self.rel_alt:.1f} / {self.takeoff_alt} m")

    def arm_response(self, response):
        if response.success:
            self.get_logger().info("Moteurs armés.")
        else:   # le message PreArm exact s'affiche dans Mission Planner
            self.get_logger().warn("Armement refusé (vérifications pré-armement ?), nouvel essai.")

    def takeoff_response(self, response):
        if response.success:
            self.get_logger().info("Décollage accepté.")
            self.step = 'montee'
        else:
            self.get_logger().warn("Décollage refusé, nouvel essai.")


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
