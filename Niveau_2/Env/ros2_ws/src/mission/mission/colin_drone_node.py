#!/usr/bin/env python3
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.time import Time  # <-- FIX: use Time for proper stamp math
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

        self.get_logger().info('Initialized node, sending to target')

        self.drone = Zenmav('tcp:127.0.0.1:5762', gps_thresh = 0.2) # Zenmav instance to access high level functions
        self.go_to_first_point() 

    def go_to_first_point(self):
        # Watch out! Takeoff and local target when wait_to_reach=True (default) are blocking calls
        # Will block messages until done, bad practice!
        self.drone.set_mode('GUIDED')
        self.drone.arm()
        self.drone.takeoff(10)
        self.drone.local_target((10, 20, -50))  # NED

        #Publish a message to signal arrival and start the challenge
        self.arrival_pub.publish(String(data='Colin'))
    

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

            # Simple constant-velocity prediction 2 s lookahead, pretty hacky solution!
            # I just did trial and error until I had a fair result
            lookahead = 2.0  # seconds
            px = self.x + dxdt * lookahead
            py = self.y + dydt * lookahead
            pz = self.z + dzdt * lookahead

            if self.follow:
                #keep ENU->NED mapping consistent when predicting ----
                point = wp(py, px, -pz, frame='local')  # (N, E, D) = (py, px, -pz)
                self.drone.global_target(point, wait_to_reach=False) #Zenmav converts local to global if needed!
                self.get_logger().info(f'PREDICT {lookahead:.1f}s → {point.coordinates} m NED')

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
            self.drone.RTL()

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
