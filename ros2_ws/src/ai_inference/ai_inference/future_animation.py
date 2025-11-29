#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
RESOLUTION = 0.1
T_OUT = 10


class FutureAnimation(Node):
    def __init__(self):
        super().__init__("future_animation")

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_occupancy",
            self.on_future,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            "/future_animation_markers",
            10
        )

        self.get_logger().info("🎞️ Future Occupancy Animation Node Started.")

    def on_future(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)
        if data.size != T_OUT * GRID_SIZE * GRID_SIZE:
            return

        future = data.reshape(T_OUT, GRID_SIZE, GRID_SIZE)

        for t in range(T_OUT):
            frame = future[t]

            ma = MarkerArray()
            now = self.get_clock().now().to_msg()

            marker_id = 0
            for y in range(GRID_SIZE):
                for x in range(GRID_SIZE):
                    v = frame[y, x]
                    if v < 0.2:
                        continue

                    m = Marker()
                    m.header.frame_id = "base_link"
                    m.header.stamp = now
                    m.id = marker_id
                    marker_id += 1

                    m.type = Marker.CUBE
                    m.action = Marker.ADD
                    m.scale.x = RESOLUTION
                    m.scale.y = RESOLUTION
                    m.scale.z = 0.02

                    m.pose.position.x = (x - GRID_SIZE/2) * RESOLUTION
                    m.pose.position.y = (y - GRID_SIZE/2) * RESOLUTION
                    m.pose.position.z = 0.03

                    m.pose.orientation.w = 1.0

                    # 빨강색으로 표시 (예측 점유)
                    m.color.r = 1.0
                    m.color.g = 0.0
                    m.color.b = 0.0
                    m.color.a = 0.8

                    ma.markers.append(m)

            self.pub.publish(ma)

            # 애니메이션 속도 (0.15초 추천)
            time.sleep(0.15)


def main(args=None):
    rclpy.init(args=args)
    node = FutureAnimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
