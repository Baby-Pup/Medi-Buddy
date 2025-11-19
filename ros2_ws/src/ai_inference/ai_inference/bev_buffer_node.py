#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray


GRID_SIZE = 256
T_IN = 10


class BevBuffer(Node):
    def __init__(self):
        super().__init__("bev_buffer")

        self.buffer = []  # list of (256*256) flattened arrays

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_frame",
            self.on_bev_frame,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_sequence",
            10
        )

        self.get_logger().info("🧰 BEV Rolling Buffer Node Started.")

    def on_bev_frame(self, msg: Float32MultiArray):
        bev = np.array(msg.data, dtype=np.float32)

        if bev.size != GRID_SIZE * GRID_SIZE:
            self.get_logger().warn("⚠ Wrong BEV shape received.")
            return

        # rolling window push
        self.buffer.append(bev)

        # keep last 10 frames
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        # 아직 10프레임 쌓이지 않았으면 패스
        if len(self.buffer) < T_IN:
            return

        # 10프레임 쌓였으면 시퀀스 publish
        seq = np.stack(self.buffer, axis=0)  # (10, 256*256)

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
