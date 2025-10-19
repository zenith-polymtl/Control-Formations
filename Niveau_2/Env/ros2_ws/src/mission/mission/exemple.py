#!/usr/bin/env python3
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.time import Time 
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
import time
from zenmav.core import Zenmav
from zenmav.zenpoint import wp


class solution(Node):
    def __init__(self):
        super().__init__('solution')

        #Définition des publishers et subscribers
        #self.nom_choisi_pub = self.create_publisher(#Type de variable ROS2, doit être importé#, '/nom_choisi', #quality of service ou queue size#)
        self.arrival_pub = self.create_publisher(String, '/arrival', 10)
        self.command_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        ##self.nom_choisi_sub = self.create_subscription(
        # #Type de variable ROS2#,
        # #doit être importé#,
        # #/nom_choisi#, 
        # #Fonction à appeler à la réception d'un message#, #quality of service ou queue size#)
        self.ballon_sub = self.create_subscription(PoseStamped, '/Ballon_pose', self.pos_callback, 10)
 
        # Initial state estimators and variables
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_stamp = None
        self.last_stamp_health = None

        self.first = True # first callback flag
        self.follow = True # following flag

        self.declare_parameter("zenmav_ip", "tcp:127.0.0.1:5762")
        zenmav_ip = (
            self.get_parameter("zenmav_ip").get_parameter_value().string_value
        )

        self.declare_parameter("takeoff_alt", 10.0)
        self.takeoff_alt = (
            self.get_parameter("takeoff_alt").get_parameter_value().double_value
        )

        self.declare_parameter("look_ahead", 2.25)
        self.look_ahead = (
            self.get_parameter("look_ahead").get_parameter_value().double_value
        )

        self.get_logger().info('Initialized node, sending to target')

        self.drone = Zenmav(zenmav_ip, gps_thresh = 2.0) # Zenmav instance to access high level functions
        self.go_to_first_point() 

    def go_to_first_point(self):
        # Watch out! Takeoff and local target when wait_to_reach=True (default) are blocking calls
        # Will block messages until done, bad practice! If used in mission, need to set wait_to_reach = False, and implement another system
        self.drone.set_mode('GUIDED')
        self.drone.arm()
        self.drone.takeoff(self.takeoff_alt)
        self.drone.local_target((10, 20, -50))  # NED

        #Publish a message to signal arrival and start the challenge
        msg = String()
        msg.data = 'Colin'
        self.arrival_pub.publish(msg)
    

    def pos_callback(self, msg: PoseStamped):
        # Message is passed in the callback accessible as msg
        # Attributes can be accessed, need to know the strucutre by looking at the message definition/ ros2 topic echo /topic
        self.ballon_stamp = msg.header.stamp
        self.x = float(msg.pose.position.x)  # E
        self.y = float(msg.pose.position.y)  # N
        self.z = float(msg.pose.position.z)  # U

        if self.last_x is None:
            
            #if self.first:
                # First sample: start health timer to check when the challenge ends
            self.health_timer = self.create_timer(5.0, self.health_check)
            self.first = False

            if self.follow:
                # ENU -> NED mapping is (N, E, D) = (y, x, -z)
                point = wp(self.y, self.x, -self.z, frame='local')
                self.drone.local_target(point, wait_to_reach=False) #non-blocking
                self.get_logger().info(f'SENDING TO {point.coordinates} m NED')
        else:
            # Time module to convert timestamps to seconds
            dt = (Time.from_msg(self.ballon_stamp) - Time.from_msg(self.last_stamp)).nanoseconds / 1e9
            if dt <= 0:
                # Out-of-order or identical stamps: just update state and skip prediction
                self.last_stamp = self.ballon_stamp
                self.last_x, self.last_y, self.last_z = self.x, self.y, self.z
                return

            # Velocities (m/s): forward difference
            dxdt = (self.x - self.last_x) / dt
            dydt = (self.y - self.last_y) / dt
            dzdt = (self.z - self.last_z) / dt

            # Simple constant-velocity prediction 2 s derivative feedforward, pretty hacky but simple solution!
            # I just did trial and error until I had a fair result
            lookahead = self.look_ahead  # seconds
            px = self.x + dxdt * lookahead
            py = self.y + dydt * lookahead
            pz = self.z + dzdt * lookahead

            if self.follow:
                #keep ENU->NED mapping consistent when predicting ----
                msg = PoseStamped()
                msg.pose.position.x = px
                msg.pose.position.y = py
                msg.pose.position.z = pz

                self.command_pub.publish(msg)

                self.get_logger().info(f'PREDICT {lookahead:.2f}s → {(py, px, -pz)} m ENU')

        # Update state for next callback
        self.last_stamp = self.ballon_stamp
        self.last_x, self.last_y, self.last_z = self.x, self.y, self.z

    def health_check(self):
        # simple watchdog: if stamp didn't advance, stop following, means Balloon pos is stopped.
        # Okay since no need to detect the drop rapidly, not ideal, a bit hacky, works for this, might need something more complex in real life
        if self.last_stamp_health != self.ballon_stamp:
            self.last_stamp_health = self.ballon_stamp
        else:
            self.follow = False
            self.drone.set_mode('RTL')

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
