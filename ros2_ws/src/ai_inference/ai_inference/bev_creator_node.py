#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


GRID_SIZE = 256
RESOLUTION = 0.1    # m per cell
CENTER = GRID_SIZE // 2


def lidar_to_bev(ranges, angle_min, angle_increment):
    bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    for i, r in enumerate(ranges):
        if np.isnan(r) or r <= 0.03 or r > 12.0:
            continue

        theta = angle_min + i * angle_increment
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        gx = int(x / RESOLUTION + CENTER)
        gy = int(y / RESOLUTION + CENTER)

        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
            bev[gy, gx] = 1.0

    return bev


class BevCreator(Node):
    def __init__(self):
        super().__init__("bev_creator")

        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.on_scan,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_frame",
            10
        )

        self.get_logger().info("📡 Real-time BEV Creator Started.")

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        bev = lidar_to_bev(
            ranges,
            msg.angle_min,
            msg.angle_increment
        )

        # flatten하여 전송
        out_msg = Float32MultiArray()
        out_msg.data = bev.flatten().tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BevCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
