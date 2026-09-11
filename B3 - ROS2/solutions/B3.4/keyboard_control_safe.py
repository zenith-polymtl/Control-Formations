#!/usr/bin/env python3
"""
Solution du bonus B3.4 : contrôle au clavier avec détection de perte de communication.

Différence avec keyboard_control.py : la callback ne commande plus le drone, elle
mémorise la commande et l'heure du message. Un timer plus rapide que le signal
(10 Hz contre 2 Hz) mesure le délai depuis le dernier message. Au-delà de la
période attendue multipliée par un facteur de sécurité, le signal est déclaré
perdu et le drone s'arrête sur place.

Pour tester : ros2 param set /teleop simulate_dropout true

À copier dans example_ws/src/b3_python_nodes/b3_python_nodes/, avec dans setup.py :
    'py_keyboard_control_safe = b3_python_nodes.keyboard_control_safe:main'
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget


class KeyboardControlSafe(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.set_up_parameters()
        self.set_up_topics()

        self.command = (0.0, 0.0, 0.0, 0.0)
        self.time_last_message = None      # aucun message reçu pour l'instant
        self.signal_lost = False

        # Timer plus rapide que le signal (10 Hz contre 2 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Contrôle clavier sécurisé démarré : signal perdu après {self.timeout:.1f} s.")

    def set_up_parameters(self):
        self.declare_parameter('horizontal_speed', 2.0)   # m/s
        self.declare_parameter('vertical_speed', 1.0)     # m/s
        self.declare_parameter('yaw_rate', 0.5)           # rad/s
        self.declare_parameter('expected_period', 0.5)    # s entre deux messages de la téléop (2 Hz)
        self.declare_parameter('safety_factor', 3.0)      # tolérance aux petites irrégularités

        self.horizontal_speed = self.get_parameter('horizontal_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        self.yaw_rate = self.get_parameter('yaw_rate').value
        self.timeout = self.get_parameter('expected_period').value * self.get_parameter('safety_factor').value

    def set_up_topics(self):
        self.key_sub = self.create_subscription(String, '/b3/teleop/key', self.key_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

    def key_callback(self, msg):
        self.time_last_message = self.get_clock().now()

        # Parsing
        key = msg.data.strip().lower()

        # Mapping : (est, nord, haut, lacet), en convention ROS (ENU)
        h = self.horizontal_speed
        v = self.vertical_speed
        r = self.yaw_rate
        mapping = {
            'w': (0.0,  h, 0.0, 0.0),
            's': (0.0, -h, 0.0, 0.0),
            'd': ( h, 0.0, 0.0, 0.0),
            'a': (-h, 0.0, 0.0, 0.0),
            'r': (0.0, 0.0,  v, 0.0),
            'f': (0.0, 0.0, -v, 0.0),
            'q': (0.0, 0.0, 0.0,  r),
            'e': (0.0, 0.0, 0.0, -r),
        }
        self.command = mapping.get(key, (0.0, 0.0, 0.0, 0.0))

    def timer_callback(self):
        # Pas encore de premier message : la téléop n'est pas démarrée, on ne
        # commande rien.
        if self.time_last_message is None:
            return

        delay = (self.get_clock().now() - self.time_last_message).nanoseconds / 1e9
        lost = delay > self.timeout

        # Un log au changement d'état seulement, pas dix par seconde
        if lost and not self.signal_lost:
            self.get_logger().warn(f"Signal perdu (aucun message depuis {delay:.1f} s) : arrêt du drone.")
        elif not lost and self.signal_lost:
            self.get_logger().info("Signal rétabli, reprise du contrôle clavier.")
        self.signal_lost = lost

        self.send_command(stop=lost)

    def send_command(self, stop=False):
        if stop:
            v_east, v_north, v_up, yaw_rate = (0.0, 0.0, 0.0, 0.0)
        else:
            v_east, v_north, v_up, yaw_rate = self.command

        # Paramètres de la commande : vitesse et vitesse de lacet (1479)
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        target.velocity.x = v_east
        target.velocity.y = v_north
        target.velocity.z = v_up
        target.yaw_rate = yaw_rate

        # Commande
        self.setpoint_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlSafe()
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
