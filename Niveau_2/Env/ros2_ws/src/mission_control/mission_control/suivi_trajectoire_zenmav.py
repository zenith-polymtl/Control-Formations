#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParamProbe(Node):
    def __init__(self):
        super().__init__('param_probe')

        # ===== DÉBUT TÂCHE 3 : Déclaration des paramètres de connexion/mission =====
        self.declare_parameter("zenmav_ip", "tcp:127.0.0.1:5762")
        self.declare_parameter("takeoff_alt", 10.0)
        self.declare_parameter("look_ahead", 2.0)               # s
        self.declare_parameter("autopublish_arrival", True)
        self.declare_parameter("arrival_name", "MonNom")
        self.declare_parameter("limit_altitude_delta", 0.0)      # 0=off
        # ===== FIN TÂCHE 3 =====


def main():
    rclpy.init()
    node = ParamProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
