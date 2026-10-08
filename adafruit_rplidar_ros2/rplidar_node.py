#!/usr/bin/env python3
# Copyright 2026 Robert Lowe <rlowe8@utm.edu>
"""ROS 2 node publishing Adafruit RPLidar scans as sensor_msgs/LaserScan."""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .adafruit_rplidar import RPLidar


class RPLidarNode(Node):
    def __init__(self):
        super().__init__("rplidar_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("range_min", 0.15)   # meters
        self.declare_parameter("range_max", 12.0)   # A1/A2: 12 m, A3: 25 m

        self.frame_id = self.get_parameter("frame_id").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value

        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.lidar = RPLidar(None, self.get_parameter("port").value, timeout=3)

        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        """iter_scans() blocks, so it lives in its own thread."""
        start = self.get_clock().now()
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                scan_data = [0.0] * 360          # fresh each revolution: no stale rays
                for _, angle, distance in scan:
                    scan_data[min(359, math.floor(angle))] = distance
                now = self.get_clock().now()
                self.publish(scan_data, start, (now - start).nanoseconds / 1e9)
                start = now
        except Exception as e:
            if self.running:
                self.get_logger().error(f"lidar read failed: {e}")

    def publish(self, scan_data, stamp, scan_time):
        msg = LaserScan()
        msg.header.stamp = stamp.to_msg()        # LaserScan stamp = time of first ray
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = math.radians(359)
        msg.angle_increment = math.radians(1)
        msg.scan_time = scan_time
        msg.time_increment = scan_time / 360
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = []
        for i in range(360):
            # RPLidar angles run clockwise; ROS (REP 103) runs counterclockwise
            d = scan_data[(360 - i) % 360] / 1000.0    # mm -> m
            # 0 means "no return"; REP 117 says report that as +inf
            ranges.append(d if self.range_min <= d <= self.range_max else math.inf)
        msg.ranges = ranges
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RPLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
