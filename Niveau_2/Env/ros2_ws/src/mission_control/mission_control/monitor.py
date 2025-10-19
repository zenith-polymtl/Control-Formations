#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sqrt
from threading import Lock
import csv
import os
from rclpy.time import Time
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
import tf_transformations
import math


def angle_diff_deg(a, b):
    if a is None or b is None:
        return None
    d = (a - b + 180.0) % 360.0 - 180.0  # in (-180, 180]
    return abs(d)


class PoseDistanceToCSV(Node):
    def __init__(self):
        super().__init__("pose_distance_to_csv")
        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8,
        )

        # ---------------- Params ----------------
        self.declare_parameter("drone_pos", "/mavros/local_position/pose")
        self.declare_parameter("Balloon_pos", "/Ballon_pose")
        self.declare_parameter("interval", 0.5)  # seconds
        self.declare_parameter("csv_path", "pose_distances.csv")
        self.declare_parameter("description", "Total distance between samples")
        self.declare_parameter("append", False)  # append vs overwrite CSV
        self.monitor_start_sub = self.create_subscription(
            String, "/monitor", self.monitor_callback, 10
        )

        drone_pos = self.get_parameter("drone_pos").get_parameter_value().string_value
        Balloon_pos = (
            self.get_parameter("Balloon_pos").get_parameter_value().string_value
        )
        self.interval = (
            self.get_parameter("interval").get_parameter_value().double_value
        )
        self.csv_path = (
            self.get_parameter("csv_path").get_parameter_value().string_value
        )
        self.description = (
            self.get_parameter("description").get_parameter_value().string_value
        )
        append = self.get_parameter("append").get_parameter_value().bool_value

        # ---------------- State ----------------
        self.actual_pos = None
        self.target_pos = None
        self.true_hdg = None
        self.target_hdg = None

        self.distance_sum = 0.0
        self.hdg_distance_total = 0.0
        self.sample_count = 0
        self.started = False

        # ---------------- Subscribers ----------------
        self.create_subscription(
            PoseStamped, drone_pos, self.actual_pos_cb, qos_profile_BE
        )
        self.create_subscription(PoseStamped, Balloon_pos, self.target_pos_cb, 10)
        self.subscription = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile_BE
        )
        self.hdg_sub = self.create_subscription(
            Float32, "/heading_target", self.hdg_target_callback, 10
        )

        # ---------------- CSV ----------------
        # If not appending, or file missing, write header
        self._csv_file = open(
            self.csv_path,
            "a" if append and os.path.exists(self.csv_path) else "w",
            newline="",
        )
        self._csv_writer = csv.writer(self._csv_file)

        # If we created a new file OR overwrote, write header
        if self._csv_file.tell() == 0:
            self._csv_writer.writerow(
                [
                    "sample_stamp_sec",
                    "sample_stamp_nanosec",
                    "actual_pos_stamp_sec",
                    "actual_pos_stamp_nanosec",
                    "actual_pos_x",
                    "actual_pos_y",
                    "actual_pos_z",
                    "target_pos_stamp_sec",
                    "target_pos_stamp_nanosec",
                    "target_pos_x",
                    "target_pos_y",
                    "target_pos_z",
                    "distance",
                    "distance_sum",
                    "hdg",
                    "hdg_target",
                    "hdg_distance",
                    "hdg_distance_total",
                ]
            )
            self._csv_file.flush()

        # ---------------- Timer ----------------
        self.create_timer(self.interval, self.sample_and_log)

        self.get_logger().info(
            f"Subscribed to '{drone_pos}' and '{Balloon_pos}', sampling every {self.interval:.3f}s, writing to '{self.csv_path}'"
        )

    def hdg_target_callback(self, msg):
        self.target_hdg = msg.data

        #self.get_logger().info(f"TARGET HDG : {self.target_hdg}")

    def imu_callback(self, msg):
        # Extract quaternion components
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        # Convert yaw to degrees
        self.true_hdg = math.degrees(yaw)

        self.get_logger().info(f"Heading: {self.true_hdg:.2f} degrees")

    def monitor_callback(self, msg: String):
        if msg.data.lower() == "go":
            self.started = True
            self.get_logger().info("Starting monitor")
        else:
            self.started = False
            self.get_logger().info("Closing monitor")
            self.destroy_node()
            rclpy.shutdown()

    # --------- Callbacks ---------
    def actual_pos_cb(self, msg: PoseStamped):
        self.actual_pos = msg

    def target_pos_cb(self, msg: PoseStamped):
        self.target_pos = msg

    def sample_and_log(self):
        if self.started:
            if self.actual_pos is None or self.target_pos is None:
                self.get_logger().info("Waiting for both poses before logging…")
                return

            # Use the node clock for a common "same" stamp at sampling time
            now = self.get_clock().now()
            sstamp_sec = int(now.nanoseconds // 1_000_000_000)
            sstamp_nsec = int(now.nanoseconds % 1_000_000_000)

            # Extract data
            p1 = self.actual_pos.pose.position
            p2 = self.target_pos.pose.position
            st1 = self.actual_pos.header.stamp
            st2 = self.target_pos.header.stamp

            dx = p1.x - p2.x
            dy = p1.y - p2.y
            dz = p1.z - p2.z
            distance = sqrt(dx * dx + dy * dy + dz * dz)
            self.distance_sum += distance
            self.sample_count += 1

            # compute heading difference using the module-level helper
            hdg_distance = angle_diff_deg(self.true_hdg, self.target_hdg)
            # angle_diff_deg returns None when one of the headings is None
            if hdg_distance is None:
                hdg_distance = 0.0
            self.hdg_distance_total += hdg_distance

            # Write row
            self._csv_writer.writerow(
                [
                    sstamp_sec,
                    sstamp_nsec,
                    st1.sec,
                    st1.nanosec,
                    p1.x,
                    p1.y,
                    p1.z,
                    st2.sec,
                    st2.nanosec,
                    p2.x,
                    p2.y,
                    p2.z,
                    f"{distance:.6f}",
                    f"{self.distance_sum:.6f}",
                    self.true_hdg,
                    self.target_hdg,
                    hdg_distance,
                    self.hdg_distance_total,
                ]
            )
            self._csv_file.flush()

            self.get_logger().debug(
                f"Sample #{self.sample_count}: d={distance:.3f} m, sum={self.distance_sum:.3f} m"
            )

    # --------- Shutdown / Summary ---------
    def destroy_node(self):
        # Append a final line with description + total
        try:
            # Blank line for readability (optional)
            self._csv_writer.writerow([])
            # Put description in first cell, total in the last cell for clarity
            self._csv_writer.writerow(
                [self.description]
                + [""] * 11
                + ["TOTAL_DISTANCE", f"{self.distance_sum:.6f}"]
            )
            self._csv_file.flush()
            self.get_logger().info(
                f"Wrote summary line: '{self.description}' — total distance = {self.distance_sum:.6f} m"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write summary line: {e}")
        finally:
            try:
                self._csv_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseDistanceToCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
