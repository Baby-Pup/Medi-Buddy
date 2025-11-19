#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
RESOLUTION = 0.1   # BEV resolution 0.1m (한 그리드 10cm)


class RiskMapToOccupancy(Node):
    def __init__(self):
        super().__init__("riskmap_to_occupancy")

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_bias/risk_map",
            self.on_risk_map,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            "/risk_map_occupancy",
            10
        )

        self.get_logger().info("🗺️ RiskMap → OccupancyGrid Converter Started")

    def on_risk_map(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)

        if data.size != GRID_SIZE * GRID_SIZE:
            self.get_logger().warn("Risk map size mismatch")
            return

        risk = data.reshape(GRID_SIZE, GRID_SIZE)

        # 0~1 risk → 0~100 cost
        occ = (risk * 100.0).astype(np.int8).flatten()

        # ===== OccupancyGrid 메시지 구성 =====
        og = OccupancyGrid()
        og.header.frame_id = "base_link"   # BEV 기준 좌표계
        og.header.stamp = self.get_clock().now().to_msg()

        og.info = MapMetaData()
        og.info.resolution = RESOLUTION
        og.info.width = GRID_SIZE
        og.info.height = GRID_SIZE
        og.info.origin.position.x = -GRID_SIZE * RESOLUTION / 2.0
        og.info.origin.position.y = -GRID_SIZE * RESOLUTION / 2.0
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        og.data = occ.tolist()

        self.pub.publish(og)


def main(args=None):
    rclpy.init(args=args)
    node = RiskMapToOccupancy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
