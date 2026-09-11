#!/usr/bin/env python3
"""
Solution B3.4 : contrôle du drone au clavier.

Reçoit la touche publiée par la téléop (2 Hz) et la traduit en consigne de
vitesse pour mavros, sur /mavros/setpoint_raw/local (message MAVLink 84,
SET_POSITION_TARGET_LOCAL_NED).

Mapping choisi (axes fixes nord et est) :
    w / s : nord / sud
    d / a : est / ouest
    r / f : monter / descendre
    q / e : tourner à gauche / à droite
    aucune touche : vitesse nulle, le drone s'arrête sur place

À copier dans example_ws/src/b3_python_nodes/b3_python_nodes/, avec dans setup.py :
    'py_keyboard_control = b3_python_nodes.keyboard_control:main'
et dans package.xml : <depend>mavros_msgs</depend>
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.set_up_parameters()
        self.set_up_topics()

        self.get_logger().info("Contrôle clavier démarré.")

    def set_up_parameters(self):
        self.declare_parameter('horizontal_speed', 2.0)   # m/s
        self.declare_parameter('vertical_speed', 1.0)     # m/s
        self.declare_parameter('yaw_rate', 0.5)           # rad/s

        self.horizontal_speed = self.get_parameter('horizontal_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        self.yaw_rate = self.get_parameter('yaw_rate').value

    def set_up_topics(self):
        self.key_sub = self.create_subscription(String, '/b3/teleop/key', self.key_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

    def key_callback(self, msg):
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
            'q': (0.0, 0.0, 0.0,  r),   # yaw_rate positif = vers la gauche
            'e': (0.0, 0.0, 0.0, -r),
        }
        # Aucune touche (ou touche inconnue) : vitesse nulle
        v_east, v_north, v_up, yaw_rate = mapping.get(key, (0.0, 0.0, 0.0, 0.0))

        # Paramètres de la commande
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Vitesse et vitesse de lacet seulement : 1 + 2 + 4 + 64 + 128 + 256 + 1024 = 1479
        target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        # mavros attend de l'ENU (x = est, y = nord, z = haut) et convertit en NED
        target.velocity.x = v_east
        target.velocity.y = v_north
        target.velocity.z = v_up
        target.yaw_rate = yaw_rate

        # Commande
        self.setpoint_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Après Ctrl+C, ROS 2 est déjà fermé : impossible d'envoyer une dernière
        # consigne. ArduPilot arrête le drone de lui-même après 3 s sans consigne.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
