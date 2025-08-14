#!/usr/bin/env python3
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
from zenmav.core import Zenmav


class solution(Node):
    def __init__(self):
        super().__init__('solution')
        self.arrival_pub = self.create_publisher(String, 'arrival' , 10)
        self.ballon_pub = self.create_subscription(PoseStamped, '/Ballon_pose', self.pos_callback)

        self.drone = Zenmav('tcp:127.0.0.1:5762')
        self.go_to_first_point()

        self.health_timer = self.create_timer()

    def go_to_first_point(self):
        self.drone.set_mode('GUIDED')
        self.drone.arm()
        self.drone.takeoff(10)
        self.drone.local_target((90, 50, -100)) #NED

        msg = String()
        msg.data = 'Colin'

        self.arrival_pub.publish(msg)

    def pos_callback(self, msg):

        self.ballon_stamp = msg.header.stamp
        self.x = msg.pose.position.x #E
        self.y = msg.pose.position.y #N
        self.z = msg.pose.position.z #U

        self.local_target((self.y, self.x, self.z))

def main():
    rclpy.init()
    node = solution()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
