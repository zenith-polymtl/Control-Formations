#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from zenmav.core import Zenmav
import numpy as np
import time


class PIDController():
    def __init__(self, kp, ki, kd, max_output = 3.0):  # Une norme de 3.0 m/s est le max pour vitesses xy envoyées au drone
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to max value
        return max(min(output, self.max_output), -self.max_output)
    
class target():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z =z

class ApproachNode(Node):
    def __init__(self):
        super().__init__("approach_node")

        self.declare_parameter('max_speed', 8)  # m/s
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.publisher_vel = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)
        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.subscriber_gt = self.create_subscription(PoseStamped, '/go_target', self.go_target_callback, qos_profile)
        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_BE)

        # PD Controllers for XYZ pos control by try and retry sim analysis
        self.pid_x = PIDController(kp=0.6, ki=0, kd=0.3, max_output=self.max_speed)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.3, max_output=self.max_speed)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.3, max_output=self.max_speed)

        # PID Controllers for XYZ pos control by Ziegler-Nichols Method       [pas fait]
        """self.pid_x = PIDController(kp=0.6, ki=0, kd=0.0)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.0)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.0)"""

        # NO PID for XYZ pos control
        """self.pid_x = PIDController(kp=1, ki=0, kd=0)
        self.pid_y = PIDController(kp=1, ki=0, kd=0)
        self.pid_z = PIDController(kp=1, ki=0, kd=0)"""

        # PID TEST DE VOL for XYZ pos control                [non modif car bien en test]
        """self.pid_x = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.3)"""

        self.curr_pos = None
        self.last_log_time_pose = 0.0
        self.last_log_time_control = 0.0
        self.atg_got_called = False
        self.approach_active = False
        self.last_time = self.get_clock().now()
        self.der_target_recu = " "
        self.i = 0
        self.before_x, self.before_y, self.before_z = None, None, None

        self.Hertz_control = 30
        self.timer = self.create_timer(1/self.Hertz_control, self.control_loop)

    def go_target_callback(self, msg):
        if self.atg_got_called == True:
            if self.curr_pos:

                self.approach_active = True
                x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

                self.der_target_time_recu = time.time()
                if not hasattr(self, 'timer_target'):
                    Hertz = 15
                    self.timer_target = self.create_timer(1/Hertz, self.Failsafe_target_acquired)

                self.target_pos = target(float(x), float(y), float(z))
                self.Failsafe_target_too_far()

                if not x == self.before_x or not y == self.before_y or not z == self.before_z:
                    self.i += 1
                    self.before_x, self.before_y, self.before_z = x, y, z

                temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
                self.get_logger().info(f"target {self.i} {self.target_pos.x, self.target_pos.y, self.target_pos.z} received at {temps}")

            else:
                self.get_logger().warn("Target received, but no position data received yet!")
    
    def Failsafe_target_too_far(self):
        if self.der_target_recu != " ":
            diff_x = self.der_target_recu.x - self.target_pos.x
            diff_y = self.der_target_recu.y - self.target_pos.y
            diff_z = self.der_target_recu.z - self.target_pos.z
            norme = (diff_x**2 + diff_y**2 + diff_z**2)**(1/2)
            if norme >= 10.0: # mètres d'écart entre target passée et actuelle
                msg = String()
                msg.data = "a.b."
                self.get_logger().warn(f"Failsafe triggered: Target received is way too far from last one : {norme:.3f} mètres. Switching to BRAKE mode.")
                self.get_logger().warn(f"diff_x: {diff_x}")
                self.get_logger().warn(f"diff_y: {diff_y}")
                self.get_logger().warn(f"diff_z: {diff_z}")
                self.abort_state_pub.publish(msg)

        self.der_target_recu = self.target_pos

    def Failsafe_target_acquired(self):
        if hasattr(self, 'der_target_time_recu'):
            elapsed = time.time() - self.der_target_time_recu
            max_time_without_target = .6 # secondes # on vise entre 0.2 et 0.8
            if elapsed >= max_time_without_target:
                self.get_logger().warn(f"Failsafe triggered: No target received in {max_time_without_target}s. Switching to BRAKE mode.")
                msg = String()
                msg.data = "a.b."
                self.abort_state_pub.publish(msg)

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message GO! received, approach starting')
            self.atg_got_called = True

    def pose_callback(self, msg):
        self.curr_pos = msg.pose.position
        current_time = time.time()
        temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))

        if current_time - self.last_log_time_pose >= 0.20:
            self.get_logger().info(f"Current position : ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f}) at {temps}")
            self.last_log_time_pose = current_time

    def control_loop(self):
        if not self.approach_active or self.curr_pos is None or self.target_pos is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = now

        error_x = self.target_pos.x - self.curr_pos.x
        error_y = self.target_pos.y - self.curr_pos.y
        error_z = self.target_pos.z - self.curr_pos.z

        vel_x = self.pid_x.compute(error_x, dt)
        vel_y = self.pid_y.compute(error_y, dt)
        vel_z = self.pid_z.compute(error_z, dt)
        
        max_output = self.pid_x.max_output
        vel_x, vel_y = self.Failsafe_max_vel(vel_x,vel_y, max_output)

        twist = TwistStamped()
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        twist.twist.linear.z = vel_z

        self.publisher_vel.publish(twist)

        current_time = time.time()
        if current_time - self.last_log_time_control >= 0.5:
            self.get_logger().info(f"PID velocities - X: {vel_x:.4f}, Y: {vel_y:.4f}, Z: {vel_z:.4f} at {current_time}")
            self.last_log_time_control = current_time

    def Failsafe_max_vel(self, vel_x,vel_y, max_output):
        eps = 0.001

        if (vel_x**2 + vel_y**2)**(1/2) >= max_output:
            hyp = (vel_x**2 + vel_y**2)**(1/2)
            theta = np.arccos(vel_x/hyp)
            vel_x = (max_output-eps) * np.cos(theta)
            vel_y = (max_output-eps) * np.sin(theta) * np.sign(vel_y)

        return vel_x, vel_y

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()   

def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()