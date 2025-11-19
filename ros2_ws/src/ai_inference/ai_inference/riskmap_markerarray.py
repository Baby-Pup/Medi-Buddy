#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
RESOLUTION = 0.1


class RiskMapMarker(Node):
    def __init__(self):
        super().__init__("riskmap_markerarray")

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_bias/risk_map",
            self.on_risk_map,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            "/future_bias/risk_markers",
            10
        )

        self.get_logger().info("🎨 RiskMap MarkerArray Visualizer Started")

    def color_from_value(self, v):
        """
        v: 0~1
        초록색 (안전) → 빨강 (위험)
        """
        v = float(np.clip(v, 0.0, 1.0))
        r = v
        g = 1.0 - v
        b = 0.0
        return r, g, b

    def on_risk_map(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)
        if data.size != GRID_SIZE * GRID_SIZE:
            return

        risk = data.reshape(GRID_SIZE, GRID_SIZE)

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        marker_id = 0

        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                v = risk[y, x]
                if v < 0.01:
                    continue  # 너무 작은 값은 skip (성능 향상)

                r, g, b = self.color_from_value(v)

                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = now
                m.id = marker_id
                marker_id += 1

                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.scale.x = RESOLUTION
                m.scale.y = RESOLUTION
                m.scale.z = 0.01

                # base_link 중심에서 좌표 계산
                m.pose.position.x = (x - GRID_SIZE/2) * RESOLUTION
                m.pose.position.y = (y - GRID_SIZE/2) * RESOLUTION
                m.pose.position.z = 0.02

                m.pose.orientation.w = 1.0

                m.color.r = r
                m.color.g = g
                m.color.b = b
                m.color.a = 0.7

                ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RiskMapMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
