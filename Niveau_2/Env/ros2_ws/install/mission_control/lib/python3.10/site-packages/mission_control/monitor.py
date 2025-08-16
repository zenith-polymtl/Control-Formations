#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sqrt
from threading import Lock
import csv
import os
from rclpy.time import Time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PoseDistanceToCSV(Node):
    def __init__(self):
        super().__init__('pose_distance_to_csv')
        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        # ---------------- Params ----------------
        self.declare_parameter('topic1', '/mavros/local_position/pose')
        self.declare_parameter('topic2', '/Ballon_pose')
        self.declare_parameter('interval', 0.5)  # seconds
        self.declare_parameter('csv_path', 'pose_distances.csv')
        self.declare_parameter('description', 'Total distance between samples')
        self.declare_parameter('append', False)  # append vs overwrite CSV
        self.monitor_start_sub = self.create_subscription(String, '/monitor', self.monitor_callback, 10)

        topic1 = self.get_parameter('topic1').get_parameter_value().string_value
        topic2 = self.get_parameter('topic2').get_parameter_value().string_value
        self.interval = self.get_parameter('interval').get_parameter_value().double_value
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.description = self.get_parameter('description').get_parameter_value().string_value
        append = self.get_parameter('append').get_parameter_value().bool_value

        # ---------------- State ----------------
        self.pose1 = None
        self.pose2 = None

        self.distance_sum = 0.0
        self.sample_count = 0
        self.started = False

        # ---------------- Subscribers ----------------
        self.create_subscription(PoseStamped, topic1, self.pose1_cb,  qos_profile_BE)
        self.create_subscription(PoseStamped, topic2, self.pose2_cb, 10)

        # ---------------- CSV ----------------
        # If not appending, or file missing, write header
        self._csv_file = open(self.csv_path, 'a' if append and os.path.exists(self.csv_path) else 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)

        # If we created a new file OR overwrote, write header
        if self._csv_file.tell() == 0:
            self._csv_writer.writerow([
                'sample_stamp_sec', 'sample_stamp_nanosec',
                'pose1_stamp_sec', 'pose1_stamp_nanosec',
                'pose1_x', 'pose1_y', 'pose1_z',
                'pose2_stamp_sec', 'pose2_stamp_nanosec',
                'pose2_x', 'pose2_y', 'pose2_z',
                'distance', 'distance_sum'
            ])
            self._csv_file.flush()

        # ---------------- Timer ----------------
        self.create_timer(self.interval, self.sample_and_log)

        self.get_logger().info(
            f"Subscribed to '{topic1}' and '{topic2}', sampling every {self.interval:.3f}s, writing to '{self.csv_path}'"
        )

    def monitor_callback(self, msg: String):
        if msg.data.lower() == 'go':
            self.started = True
            self.get_logger().info("Starting monitor")
        else:
            self.started = False
            self.get_logger().info("Closing monitor")
            self.destroy_node()
            rclpy.shutdown()


    # --------- Callbacks ---------
    def pose1_cb(self, msg: PoseStamped):
        self.pose1 = msg

    def pose2_cb(self, msg: PoseStamped):
        self.pose2 = msg

    def sample_and_log(self):
        if self.started:
            if self.pose1 is None or self.pose2 is None:
                self.get_logger().info("Waiting for both poses before logging…")
                return

            # Use the node clock for a common "same" stamp at sampling time
            now = self.get_clock().now()
            sstamp_sec = int(now.nanoseconds // 1_000_000_000)
            sstamp_nsec = int(now.nanoseconds % 1_000_000_000)

            # Extract data
            p1 = self.pose1.pose.position
            p2 = self.pose2.pose.position
            st1 = self.pose1.header.stamp
            st2 = self.pose2.header.stamp

            dx = p1.x - p2.x
            dy = p1.y - p2.y
            dz = p1.z - p2.z
            distance = sqrt(dx*dx + dy*dy + dz*dz)
            self.distance_sum += distance
            self.sample_count += 1

            # Write row
            self._csv_writer.writerow([
                sstamp_sec, sstamp_nsec,
                st1.sec, st1.nanosec, p1.x, p1.y, p1.z,
                st2.sec, st2.nanosec, p2.x, p2.y, p2.z,
                f"{distance:.6f}", f"{self.distance_sum:.6f}"
            ])
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
            self._csv_writer.writerow([self.description] + [''] * 11 + ['TOTAL_DISTANCE', f"{self.distance_sum:.6f}"])
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


if __name__ == '__main__':
    main()
