#!/usr/bin/env python3
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

# ----- User-supplied parameters/code -----
MAX_SPEED     = 20.0   # m/s (hard cap)
MAX_ACCEL     = 5.0    # m/s^2 (hard cap)
MARGIN        = 0.995  # safety factor vs theory bounds
T             = 130.0  # duration (s)
DT            = 0.02   # timestep (s)
Z0            = 0.0   # altitude offset (m)
SEED          = 116    # RNG seed (change for a different “random” path)

# Component counts per axis (more = richer motion)
NX, NY, NZ = 12, 12, 4

# Random ranges (feel free to tweak)
# Frequencies in Hz (periods ~50–1000 s); vertical a tad slower
FX_RANGE = (0.0002, 0.003)
FY_RANGE = (0.0001, 0.0017)
FZ_RANGE = (0.0015, 0.010)

# Amplitude ranges in meters (horizontal > vertical)
AX_RANGE = (15.0, 60.0)
AY_RANGE = (12.0, 50.0)
AZ_RANGE = (4.0,  18.0)

def rand_components(n, frange, arange, rng):
    f  = rng.uniform(*frange, size=n)     # Hz
    A  = rng.uniform(*arange, size=n)     # m
    ph = rng.uniform(0, 2*np.pi, size=n)  # rad
    return A, f, ph

def build_soft_trajectory(t):
    rng = np.random.default_rng(SEED)

    Ax, fx, phx = rand_components(NX, FX_RANGE, AX_RANGE, rng)
    Ay, fy, phy = rand_components(NY, FY_RANGE, AY_RANGE, rng)
    Az, fz, phz = rand_components(NZ, FZ_RANGE, AZ_RANGE, rng)

    wx, wy, wz = 2*np.pi*fx, 2*np.pi*fy, 2*np.pi*fz

    # Worst-case bounds (independent of phases)
    # For s(t) = S * Σ A_i sin(w_i t + φ_i):
    #  |v|_bound = S * sqrt( (Σ A_x w_x)^2 + (Σ A_y w_y)^2 + (Σ A_z w_z)^2 )
    #  |a|_bound = S * sqrt( (Σ A_x w_x^2)^2 + (Σ A_y w_y^2)^2 + (Σ A_z w_z^2)^2 )
    Dx  = np.sum(Ax*wx);   Dy  = np.sum(Ay*wy);   Dz  = np.sum(Az*wz)
    Bv  = np.sqrt(Dx**2 + Dy**2 + Dz**2)

    D2x = np.sum(Ax*wx**2); D2y = np.sum(Ay*wy**2); D2z = np.sum(Az*wz**2)
    Ba  = np.sqrt(D2x**2 + D2y**2 + D2z**2)

    S_speed = (MARGIN*MAX_SPEED) / max(Bv, 1e-12)
    S_accel = (MARGIN*MAX_ACCEL) / max(Ba, 1e-12)
    S0 = min(S_speed, S_accel)

    def mix_sin(a, w, ph):    return (a[None,:]*np.sin(w[None,:]*t[:,None] + ph[None,:])).sum(axis=1)
    def mix_cos_w(a, w, ph):  return (a[None,:]*w[None,:]*np.cos(w[None,:]*t[:,None] + ph[None,:])).sum(axis=1)
    def mix_sin_w2(a, w, ph): return (a[None,:]*w[None,:]**2*np.sin(w[None,:]*t[:,None] + ph[None,:])).sum(axis=1)

    # Position / velocity / acceleration (analytic)
    x  = S0*mix_sin(Ax, wx, phx);   y  = S0*mix_sin(Ay, wy, phy);   z  = Z0 + S0*mix_sin(Az, wz, phz)
    vx = S0*mix_cos_w(Ax, wx, phx); vy = S0*mix_cos_w(Ay, wy, phy); vz = S0*mix_cos_w(Az, wz, phz)
    ax = -S0*mix_sin_w2(Ax, wx, phx); ay = -S0*mix_sin_w2(Ay, wy, phy); az = -S0*mix_sin_w2(Az, wz, phz)

    vmag = np.sqrt(vx**2 + vy**2 + vz**2)
    amag = np.sqrt(ax**2 + ay**2 + az**2)

    # Gentle nudge toward (but below) the caps
    target_v = 0.98*MAX_SPEED
    target_a = 0.90*MAX_ACCEL
    k_speed_theory = (MARGIN*MAX_SPEED)/(S0*Bv) if Bv>0 else 1.0
    k_accel_theory = (MARGIN*MAX_ACCEL)/(S0*Ba) if Ba>0 else 1.0
    k_speed_sample = (target_v/vmag.max()) if vmag.max()>0 else 1.0
    k_accel_sample = (target_a/amag.max()) if amag.max()>0 else 1.0
    k = min(k_speed_theory, k_accel_theory, k_speed_sample, k_accel_sample)

    if k != 1.0:
        x, y, z  = k*x, k*y, Z0 + k*(z - Z0)
        vx,vy,vz = k*vx, k*vy, k*vz
        ax,ay,az = k*ax, k*ay, k*az
        vmag     = np.sqrt(vx**2 + vy**2 + vz**2)
        amag     = np.sqrt(ax**2 + ay**2 + az**2)

    v_bound = (S0*k)*Bv
    a_bound = (S0*k)*Ba
    return x,y,z

class Ballon_publisher(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(String, 'arrival' , self.callback, 10)
        self.monitor_start_pub = self.create_publisher(String, 'monitor', 10)
        self.ballon_pub = self.create_publisher(PoseStamped, '/Ballon_pose', 10)

        self.begin_position = (10, 20, -50)
        self.get_logger().info(f'Initialized node, wainting for arrival')


    def callback(self, msg: String):
        self.get_logger().info(f"Arrival of : {msg.data}, starting ballon run, scoring starting in 10 s")

        self.name = msg.data
        self.scoring_start_timer = self.create_timer(10, self.monitor_start_callback)
        self.ballon_traj_timer = self.create_timer(0.05, self.ballon_publisher)
        self.start_time = time.time()

    def ballon_publisher(self):

        while time.time()- self.start_time < T:

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            t = np.array([time.time() - self.start_time])
            self.x, self.y, self.z = build_soft_trajectory(t)
            msg.pose.position.x = float(self.x) + float(self.begin_position[1])
            msg.pose.position.y = float(self.y) + float(self.begin_position[0])
            msg.pose.position.z = float(self.z) - float(self.begin_position[2])

            self.ballon_pub.publish(msg)

    def monitor_start_callback(self):
        msg = String()
        msg.data = 'Monitoring started... Evaluating performance'
        self.monitor_start_pub.publish(msg)
        self.destroy_timer(self.scoring_start_timer)

def main():
    rclpy.init()
    node = Ballon_publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
