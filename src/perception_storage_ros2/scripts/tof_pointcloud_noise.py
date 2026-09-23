#!/usr/bin/env python3
"""Inject radial measurement noise into a Gazebo ToF PointCloud2 stream."""

import copy
import math
import random
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class TofPointCloudNoise(Node):
    def __init__(self):
        super().__init__('tof_pointcloud_noise')
        self.declare_parameter('range_stddev_m', 0.01)
        self.declare_parameter('range_bias_m', 0.0)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('seed', 20260915)
        self.stddev = float(self.get_parameter('range_stddev_m').value)
        self.bias = float(self.get_parameter('range_bias_m').value)
        self.dropout = float(self.get_parameter('dropout_probability').value)
        self.rng = random.Random(int(self.get_parameter('seed').value))
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PointCloud2, 'points_out', 5)
        self.sub = self.create_subscription(PointCloud2, 'points_in', self.callback, qos)

    def callback(self, msg):
        x_field = next((f for f in msg.fields if f.name == 'x'), None)
        y_field = next((f for f in msg.fields if f.name == 'y'), None)
        z_field = next((f for f in msg.fields if f.name == 'z'), None)
        if not x_field or not y_field or not z_field or msg.point_step < 12:
            self.pub.publish(msg)
            return
        out = copy.deepcopy(msg)
        data = bytearray(msg.data)
        endian = '>' if msg.is_bigendian else '<'
        fmt = endian + 'f'
        total = msg.width * msg.height
        for index in range(total):
            base = (index // msg.width) * msg.row_step + (index % msg.width) * msg.point_step
            offsets = (base + x_field.offset, base + y_field.offset, base + z_field.offset)
            x, y, z = (struct.unpack_from(fmt, data, offset)[0] for offset in offsets)
            radius = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(radius) or radius <= 1e-6:
                continue
            if self.dropout > 0.0 and self.rng.random() < self.dropout:
                value = float('nan')
                for offset in offsets:
                    struct.pack_into(fmt, data, offset, value)
                continue
            measured = max(0.0, radius + self.bias + self.rng.gauss(0.0, self.stddev))
            scale = measured / radius
            struct.pack_into(fmt, data, offsets[0], x * scale)
            struct.pack_into(fmt, data, offsets[1], y * scale)
            struct.pack_into(fmt, data, offsets[2], z * scale)
        out.data = data
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TofPointCloudNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
