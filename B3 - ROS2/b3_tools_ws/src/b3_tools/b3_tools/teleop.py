#!/usr/bin/env python3
"""
Téléop clavier de la formation B3 (node fournie).

Lit les touches w a s d q e r f dans le terminal et publie, à fréquence fixe
(2 Hz par défaut), la touche active sur un topic std_msgs/String. Une touche
reste active tant qu'on la maintient enfoncée ; sans touche, le message contient
une chaîne vide. La publication commence dès le démarrage.

La node ne donne aucun sens aux touches : traduire une touche en mouvement du
drone est le travail de la node de contrôle écrite en B3.

Le paramètre simulate_dropout coupe la publication de 5 à 10 s toutes les 30 s,
pour le bonus perte de communication. Il est relu à chaque période, donc il se
change pendant que la node roule :
    ros2 param set /teleop simulate_dropout true

Le clavier est lu dans le terminal où la node a été lancée (/dev/tty), avec
ros2 run comme avec ros2 launch. L'entrée standard ne suffirait pas : ros2 launch
la remplace par un tube pour les nodes qu'il démarre.
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
from std_msgs.msg import String

KEYS = 'wasdqerf'

HELP = """
---------------------------------------------
 Téléop clavier B3
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
        self.key_pub = self.create_publisher(String, self.topic_name, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info(f"Téléop démarrée : touches publiées à {self.rate} Hz sur {self.topic_name}")

    # ------------------------------------------------------------------
    # Paramètres et état
    # ------------------------------------------------------------------
    def set_up_parameters(self):
        self.declare_parameter('topic_name', '/b3/teleop/key')
        self.declare_parameter('rate', 2.0)                 # Hz
        self.declare_parameter('hold_timeout', 0.8)         # s sans répétition avant de considérer la touche relâchée
        self.declare_parameter('simulate_dropout', False)   # relu à chaque période
        self.declare_parameter('dropout_period', 30.0)      # s entre deux coupures
        self.declare_parameter('dropout_min', 5.0)          # durée minimale d'une coupure (s)
        self.declare_parameter('dropout_max', 10.0)         # durée maximale d'une coupure (s)

        self.topic_name = self.get_parameter('topic_name').value
        self.rate = self.get_parameter('rate').value
        self.hold_timeout = self.get_parameter('hold_timeout').value
        self.dropout_period = self.get_parameter('dropout_period').value
        self.dropout_min = self.get_parameter('dropout_min').value
        self.dropout_max = self.get_parameter('dropout_max').value

    def define_initial_state(self):
        self.last_key = ''
        self.last_key_time = 0.0
        self.published_key = None
        self.next_dropout = None
        self.dropout_end = 0.0

    # ------------------------------------------------------------------
    # Clavier (thread séparé : la lecture ne doit pas bloquer rclpy.spin)
    # ------------------------------------------------------------------
    def read_keys(self, fd):
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
    # Simulation de coupure (bonus)
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
        if self.dropout_active():
            return

        key = self.current_key()
        self.key_pub.publish(String(data=key))

        if key != self.published_key:
            self.get_logger().info(f"Touche : {key if key else '(aucune)'}")
            self.published_key = key


def main(args=None):
    # Le terminal où la node a été lancée, qu'on passe par ros2 run ou ros2 launch
    try:
        tty_fd = os.open('/dev/tty', os.O_RDONLY)
    except OSError:
        print("Aucun terminal : lancer la téléop depuis un terminal (ros2 run ou ros2 launch).",
              file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = Teleop()
    print(HELP, flush=True)

    # Mode cbreak : chaque touche arrive tout de suite, sans attendre Entrée et
    # sans s'afficher. Ctrl+C fonctionne toujours.
    settings = termios.tcgetattr(tty_fd)
    tty.setcbreak(tty_fd)
    threading.Thread(target=node.read_keys, args=(tty_fd,), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, settings)
        os.close(tty_fd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
