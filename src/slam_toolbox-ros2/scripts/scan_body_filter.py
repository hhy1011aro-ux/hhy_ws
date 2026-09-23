#!/usr/bin/env python3
"""Remove returns that lie inside the simulated HL2 body footprint."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanBodyFilter(Node):
    def __init__(self):
        super().__init__('scan_body_filter')
        self.declare_parameter('base_to_lidar_x', 0.445)
        self.declare_parameter('base_to_lidar_y', -0.245)
        self.declare_parameter('base_to_lidar_yaw', -0.0413)
        self.declare_parameter(
            'body_boxes',
            [-0.55, 0.55, -0.34, 0.34,
             -0.55, 0.55, -1.75, -1.00])
        self.declare_parameter('replace_with_inf', True)

        self.tx = float(self.get_parameter('base_to_lidar_x').value)
        self.ty = float(self.get_parameter('base_to_lidar_y').value)
        self.yaw = float(self.get_parameter('base_to_lidar_yaw').value)
        values = [float(v) for v in self.get_parameter('body_boxes').value]
        if len(values) % 4 != 0 or not values:
            raise ValueError('body_boxes must contain groups of xmin,xmax,ymin,ymax')
        self.boxes = [tuple(values[i:i + 4]) for i in range(0, len(values), 4)]
        self.replace_with_inf = bool(self.get_parameter('replace_with_inf').value)
        self.cos_yaw = math.cos(self.yaw)
        self.sin_yaw = math.sin(self.yaw)

        # pointcloud_to_laserscan publishes with SensorDataQoS (best effort).
        # Use the same policy on the input or DDS will report an incompatible
        # QoS and the filter will receive no scans.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(LaserScan, 'scan_out', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan_in', self.callback, sensor_qos)

    def inside_body(self, range_value, angle):
        if not math.isfinite(range_value):
            return False
        x_lidar = range_value * math.cos(angle)
        y_lidar = range_value * math.sin(angle)
        x_base = self.tx + self.cos_yaw * x_lidar - self.sin_yaw * y_lidar
        y_base = self.ty + self.sin_yaw * x_lidar + self.cos_yaw * y_lidar
        return any(
            xmin <= x_base <= xmax and ymin <= y_base <= ymax
            for xmin, xmax, ymin, ymax in self.boxes)

    def callback(self, scan):
        filtered = scan
        filtered.ranges = list(scan.ranges)
        for index, value in enumerate(filtered.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if self.inside_body(value, angle):
                filtered.ranges[index] = (
                    float('inf') if self.replace_with_inf else scan.range_max)
        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBodyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
