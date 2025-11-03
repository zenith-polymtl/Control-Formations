import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from zenmav.core import Zenmav
from zenmav.zenpoint import wp
from std_msgs.msg import String

class GuillaumeNode(Node):
    def __init__(self):
        super().__init__('guillaume_node')

        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.name_publisher = self.create_publisher(String, '/arrival', 10)

        self.ballon_sub = self.create_subscription(PoseStamped, '/Ballon_pose', self.ballon_pos_callback, 10)

        self.go_to_initial_point()

    def ballon_pos_callback(self, ballon_pos: PoseStamped):
        self.pos_publisher.publish(ballon_pos)

    def go_to_initial_point(self):
        self.get_logger().info('zenmav init')
        drone = Zenmav(ip = 'tcp:127.0.0.1:5762')
        drone.set_mode('GUIDED')
        drone.set_param("ANGLE_MAX", 5000)
        drone.set_param("WPNAV_SPEED", 1800)
        drone.arm()
        self.get_logger().info('drone armed')
        drone.takeoff(altitude = 2)
        self.get_logger().info('takeoff done')
        local_point = wp(10, 20, -50, frame = "local")
        drone.local_target(local_point)
    
def main():
    rclpy.init()
    node = GuillaumeNode()

    msg = String()
    msg.data = 'Guillaume'
    node.name_publisher.publish(msg)

    print("Node starts")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()