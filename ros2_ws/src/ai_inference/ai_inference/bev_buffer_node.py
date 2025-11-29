#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 128
T_IN = 8


class BevBuffer(Node):
    def __init__(self):
        super().__init__("bev_buffer")

        self.buffer = []

        self.sub = self.create_subscription(
            Float32MultiArray, "/bev_frame", self.on_bev, 10
        )
        self.pub = self.create_publisher(
            Float32MultiArray, "/bev_pose_sequence", 10
        )

        self.get_logger().info("🧰 BEV Buffer Started (8 frames)")

    def on_bev(self, msg: Float32MultiArray):
        bev = np.array(msg.data, dtype=np.float32)

        if bev.size != GRID_SIZE * GRID_SIZE:
            self.get_logger().warn(f"Invalid BEV size: {bev.size}")
            return

        self.buffer.append(bev)

        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        if len(self.buffer) < T_IN:
            return

        seq = np.stack(self.buffer, axis=0)
        out = Float32MultiArray()
        out.data = seq.flatten().tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BevBuffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
