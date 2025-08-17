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
        self.arrival_pub = self.create_publisher(String, '/arrival', 10)
        self.ballon_sub = self.create_subscription(PoseStamped, '/Ballon_pose', self.pos_callback, 10)
        self.hdg_sub = self.create_subscription(Float32, '/heading_target', self.hdg_target_callback, 10)
 
        # State
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_stamp = None
        self.last_stamp_health = None

        self.first = True
        self.follow = True

        self.get_logger().info('Initialized node, sending to target')

        self.drone = Zenmav('tcp:127.0.0.1:5762', gps_thresh = 0.2)
        self.go_to_first_point()

    def go_to_first_point(self):
        self.drone.set_mode('GUIDED')
        self.drone.arm()
        self.drone.takeoff(10)
        self.drone.local_target((10, 20, -50))  # NED

        self.arrival_pub.publish(String(data='Colin'))
    
    def hdg_target_callback(self, msg):
        self.target_hdg = msg.data
        self.get_logger().info(f'TARGET HDG : {self.target_hdg}')

    def health_check(self):
        # simple watchdog: if stamp didn't advance, stop following, means Ballon pos is stopped
        if self.last_stamp_health != self.ballon_stamp:
            self.last_stamp_health = self.ballon_stamp
        else:
            self.follow = False
            self.drone.RTL()

    def pos_callback(self, msg: PoseStamped):
        self.ballon_stamp = msg.header.stamp
        self.x = float(msg.pose.position.x)  # E
        self.y = float(msg.pose.position.y)  # N
        self.z = float(msg.pose.position.z)  # U

        if self.last_x is None:
            # First sample: start health timer and go to current pose
            if self.first:
                self.health_timer = self.create_timer(5.0, self.health_check)
                self.first = False

            if self.follow:
                # ENU -> NED mapping is (N, E, D) = (y, x, -z)
                point = wp(self.y, self.x, -self.z, frame='local')
                self.drone.local_target(point, wait_to_reach=False)
                self.get_logger().info(f'SENDING TO {point.coordinates} m NED')
        else:
            # ---- FIX: proper dt in seconds (current - last) using rclpy.Time ----
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

            # Simple constant-velocity prediction 2 s ahead, bad fix but good enough to show the point
            lookahead = 2.0  # seconds
            px = self.x + dxdt * lookahead
            py = self.y + dydt * lookahead
            pz = self.z + dzdt * lookahead

            if self.follow:
                # ---- FIX: keep ENU->NED mapping consistent when predicting ----
                point = wp(py, px, -pz, frame='local')  # (N, E, D) = (py, px, -pz)
                self.drone.global_target(point, wait_to_reach=False)
                self.drone.connection.mav.command_long_send(  
                    self.drone.connection.target_system,    # target_system  
                    self.drone.connection.target_component, # target_component  
                    115,                        # MAV_CMD_CONDITION_YAW  
                    0,                          # confirmation  
                    self.target_hdg,               # param1: target angle (0-360 degrees)  
                    45,              # param2: angular speed (deg/s)  
                    0,                  # param3: direction (-1 or 1)  
                    1,                   # param4: relative (0) or absolute (1)  
                    0,                          # param5: empty  
                    0,                          # param6: empty  
                    0                           # param7: empty  
                )
                self.get_logger().info(f'PREDICT {lookahead:.1f}s → {point.coordinates} m NED')

        # Update state for next callback
        self.last_stamp = self.ballon_stamp
        self.last_x, self.last_y, self.last_z = self.x, self.y, self.z


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
