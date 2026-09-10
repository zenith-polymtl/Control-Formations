#!/usr/bin/env python3
"""
Téléop clavier de la formation B3.3 (node fournie).

Lit les touches w a s d q e r f dans le terminal et publie, à fréquence fixe
(2 Hz par défaut), la touche active sur un topic std_msgs/String. Une touche
reste active tant qu'on la maintient enfoncée ; sans touche, le message contient
une chaîne vide.

La node ne donne aucun sens aux touches : traduire une touche en mouvement du
drone est le travail de la node de contrôle écrite en B3.3.

Par défaut, rien n'est publié avant la fin du décollage (topic
/b3/takeoff/completed, publié par la node takeoff) : en GUIDED, une commande de
mouvement reçue pendant la montée remplacerait le décollage.

Le paramètre simulate_dropout coupe la publication de 5 à 10 s toutes les 30 s,
pour le bonus de B3.3 (perte de communication). Il est relu à chaque période,
donc il se change pendant que la node roule :
    ros2 param set /teleop simulate_dropout true

Doit rouler dans un terminal interactif (ros2 run b3_tools teleop), jamais dans
un fichier launch : ros2 launch ne donne pas le clavier aux nodes qu'il démarre.
"""

import os
import random
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

KEYS = 'wasdqerf'

HELP = """
---------------------------------------------
 Téléop clavier B3.3
---------------------------------------------
 Touches : w a s d q e r f
 Maintenir une touche pour la garder active.
 Relâcher : plus aucune touche (message vide).
 Ctrl+C : quitter
---------------------------------------------
"""


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        self.set_up_parameters()
        self.define_initial_state()
        self.set_up_topics()

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(f"Téléop démarrée : touches publiées à {self.rate} Hz sur {self.topic_name}")
        if not self.takeoff_done:
            self.get_logger().info(
                f"En attente de la fin du décollage ({self.takeoff_topic}). "
                "Passer le drone en GUIDED dans Mission Planner pour lancer la node takeoff. "
                "Pour publier tout de suite : --ros-args -p wait_takeoff:=false")

    # ------------------------------------------------------------------
    # Paramètres et état
    # ------------------------------------------------------------------
    def set_up_parameters(self):
        self.declare_parameter('topic_name', '/b3/teleop/key')
        self.declare_parameter('rate', 2.0)                 # Hz
        self.declare_parameter('hold_timeout', 0.8)         # s sans répétition avant de considérer la touche relâchée
        self.declare_parameter('wait_takeoff', True)
        self.declare_parameter('takeoff_topic', '/b3/takeoff/completed')
        self.declare_parameter('simulate_dropout', False)   # relu à chaque période
        self.declare_parameter('dropout_period', 30.0)      # s entre deux coupures
        self.declare_parameter('dropout_min', 5.0)          # durée minimale d'une coupure (s)
        self.declare_parameter('dropout_max', 10.0)         # durée maximale d'une coupure (s)

        self.topic_name = self.get_parameter('topic_name').value
        self.rate = self.get_parameter('rate').value
        self.hold_timeout = self.get_parameter('hold_timeout').value
        self.wait_takeoff = self.get_parameter('wait_takeoff').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.dropout_period = self.get_parameter('dropout_period').value
        self.dropout_min = self.get_parameter('dropout_min').value
        self.dropout_max = self.get_parameter('dropout_max').value

    def define_initial_state(self):
        self.takeoff_done = not self.wait_takeoff
        self.last_key = ''
        self.last_key_time = 0.0
        self.published_key = None
        self.next_dropout = None
        self.dropout_end = 0.0

    # ------------------------------------------------------------------
    # Topics
    # ------------------------------------------------------------------
    def set_up_topics(self):
        self.key_pub = self.create_publisher(String, self.topic_name, 10)

        # TRANSIENT_LOCAL : le dernier message publié est gardé et livré aux
        # subscribers qui arrivent en retard. La téléop peut donc démarrer après
        # la fin du décollage et l'apprendre quand même.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.takeoff_sub = self.create_subscription(
            Bool, self.takeoff_topic, self.takeoff_callback, latched_qos)

    def takeoff_callback(self, msg: Bool):
        if msg.data and not self.takeoff_done:
            self.takeoff_done = True
            self.get_logger().info("Décollage terminé : publication des touches.")

    # ------------------------------------------------------------------
    # Clavier (thread séparé : la lecture ne doit pas bloquer rclpy.spin)
    # ------------------------------------------------------------------
    def read_keys(self):
        fd = sys.stdin.fileno()
        while rclpy.ok():
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            text = os.read(fd, 64).decode(errors='ignore')
            # Les flèches envoient une séquence d'échappement (\x1b[A...) dont les
            # lettres ne doivent pas être prises pour des touches.
            text = text.split('\x1b')[0].lower()
            keys = [c for c in text if c in KEYS]
            if keys:
                self.last_key = keys[-1]
                self.last_key_time = time.monotonic()

    def current_key(self):
        # Maintenir une touche la répète (répétition automatique du clavier). Si
        # aucune répétition n'arrive pendant hold_timeout, la touche est relâchée.
        if time.monotonic() - self.last_key_time <= self.hold_timeout:
            return self.last_key
        return ''

    # ------------------------------------------------------------------
    # Simulation de coupure (bonus B3.3)
    # ------------------------------------------------------------------
    def dropout_active(self):
        """Vrai pendant une coupure simulée."""
        if not self.get_parameter('simulate_dropout').value:
            self.next_dropout = None
            return False

        now = time.monotonic()
        if self.next_dropout is None:
            # La simulation vient d'être activée
            self.next_dropout = now + self.dropout_period
        if now >= self.next_dropout:
            # Début d'une coupure, et rendez-vous pour la suivante
            duration = random.uniform(self.dropout_min, self.dropout_max)
            self.dropout_end = now + duration
            self.next_dropout = now + self.dropout_period
            self.get_logger().warn(f"Coupure simulée : plus aucun message pendant {duration:.1f} s.")
        return now < self.dropout_end

    # ------------------------------------------------------------------
    # Publication
    # ------------------------------------------------------------------
    def timer_callback(self):
        if not self.takeoff_done:
            return
        if self.dropout_active():
            return

        key = self.current_key()
        self.key_pub.publish(String(data=key))

        if key != self.published_key:
            self.get_logger().info(f"Touche : {key if key else '(aucune)'}")
            self.published_key = key


def main(args=None):
    if not sys.stdin.isatty():
        print("La téléop doit rouler dans un terminal interactif : ros2 run b3_tools teleop "
              "(pas dans un fichier launch).", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = Teleop()
    print(HELP)

    # Mode cbreak : chaque touche arrive tout de suite, sans attendre Entrée et
    # sans s'afficher. Ctrl+C fonctionne toujours.
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    threading.Thread(target=node.read_keys, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
