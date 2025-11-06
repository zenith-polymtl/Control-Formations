import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from zenmav.core import Zenmav
from zenmav.zenpoint import wp
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import time

class GuillaumeNode(Node):
    def __init__(self):
        super().__init__('guillaume_node')

        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.name_publisher = self.create_publisher(String, '/arrival', 10)

        self.ballon_sub = self.create_subscription(PoseStamped, '/Ballon_pose', self.ballon_pos_callback, 1)
        self.last_ballon_pos = None
        self.last_ballon_speed = None
        self.last_time = None

        self.go_to_initial_point()

    def ballon_pos_callback(self, ballon_pos: PoseStamped):
        
        t = time.time()
        
        if self.last_ballon_pos is None:
            self.last_ballon_pos = ballon_pos.pose.position
            self.last_time = t
            self.pos_publisher.publish(ballon_pos)
            return
        
        dt = t - self.last_time

        dx = ballon_pos.pose.position.x - self.last_ballon_pos.x
        dy = ballon_pos.pose.position.y - self.last_ballon_pos.y
        dz = ballon_pos.pose.position.z - self.last_ballon_pos.z

        speed = np.array([dx, dy, dz]) / dt

        if self.last_ballon_speed is None:
            self.last_ballon_pos = ballon_pos.pose.position
            self.last_time = t
            self.last_ballon_speed = speed
            self.pos_publisher.publish(ballon_pos)
            return

        ddx = speed[0] - self.last_ballon_speed[0]
        ddy = speed[1] - self.last_ballon_speed[1]
        ddz = speed[2] - self.last_ballon_speed[2]

        acceleration = np.array([ddx, ddy, ddz]) / dt

        prediction = PoseStamped()

        # heuristic is used to make the drone faster
        heuristic = 10
        prediction.pose.position.x = ballon_pos.pose.position.x + (speed[0] * dt + 0.5 * acceleration[0] * dt*dt) * heuristic
        prediction.pose.position.y = ballon_pos.pose.position.y + (speed[1] * dt + 0.5 * acceleration[1] * dt*dt) * heuristic
        prediction.pose.position.z = ballon_pos.pose.position.z + (speed[2] * dt + 0.5 * acceleration[2] * dt*dt) * heuristic

        self.last_time = t
        self.last_ballon_pos = ballon_pos.pose.position
        self.last_ballon_speed = speed
        self.pos_publisher.publish(prediction)
        

    def go_to_initial_point(self):
        self.get_logger().info('zenmav init')
        self.drone = Zenmav(ip = 'tcp:127.0.0.1:5762')
        self.drone.set_mode('GUIDED')
        self.drone.set_param("ANGLE_MAX", 5000)
        self.drone.set_param("WPNAV_SPEED", 1800)
        self.drone.arm()
        self.get_logger().info('drone armed')
        self.drone.takeoff(altitude = 2)
        self.get_logger().info('takeoff done')
        local_point = wp(10, 20, -50, frame = "local")
        self.drone.local_target(local_point)
        self.get_logger().info("arrived at position")
    
def main():
    rclpy.init()
    node = GuillaumeNode()
    
    node.get_logger().info("publishing name...")
    msg = String()
    msg.data = 'Guillaume'
    node.name_publisher.publish(msg)

    print("Node starts")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drone.RTL()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()