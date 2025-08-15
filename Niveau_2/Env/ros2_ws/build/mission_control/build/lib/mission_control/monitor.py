#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sqrt
from threading import Lock

class PoseDistanceCollector(Node):
    def __init__(self):
        super().__init__('pose_distance_collector')

        # Parameters
        self.declare_parameter('topic1', '/pose1')
        self.declare_parameter('topic2', '/Ballon_pose')
        self.declare_parameter('interval', 0.5)  # seconds

        topic1 = self.get_parameter('topic1').get_parameter_value().string_value
        topic2 = self.get_parameter('topic2').get_parameter_value().string_value
        self.interval = self.get_parameter('interval').get_parameter_value().double_value

        # Variables
        self.pose1 = None
        self.pose2 = None
        self.lock = Lock()
        self.distance_sum = 0.0

        # Subscribers
        self.create_subscription(PoseStamped, topic1, self.pose1_callback, 10)
        self.create_subscription(PoseStamped, topic2, self.pose2_callback, 10)

        # Timer for sampling
        self.create_timer(self.interval, self.sample_and_compute)

        self.get_logger().info(f"Subscribed to '{topic1}' and '{topic2}', sampling every {self.interval}s")

    def pose1_callback(self, msg: PoseStamped):
        with self.lock:
            self.pose1 = msg

    def pose2_callback(self, msg: PoseStamped):
        with self.lock:
            self.pose2 = msg

    def sample_and_compute(self):
        with self.lock:
            if self.pose1 is None or self.pose2 is None:
                self.get_logger().warn("Waiting for both poses to be received...")
                return

            # For matching "on the same stamp" — here we just log the most recent stamp for each
            stamp1 = self.pose1.header.stamp
            stamp2 = self.pose2.header.stamp

            # Compute Euclidean distance
            dx = self.pose1.pose.position.x - self.pose2.pose.position.x
            dy = self.pose1.pose.position.y - self.pose2.pose.position.y
            dz = self.pose1.pose.position.z - self.pose2.pose.position.z
            distance = sqrt(dx*dx + dy*dy + dz*dz)

            self.distance_sum += distance

            self.get_logger().info(
                f"[{stamp1.sec}.{stamp1.nanosec}] vs [{stamp2.sec}.{stamp2.nanosec}] "
                f"Distance: {distance:.3f} m | Sum: {self.distance_sum:.3f} m"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PoseDistanceCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
