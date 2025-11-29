#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 128
RESOLUTION = 0.1

class RiskMapVisualizer(Node):
    def __init__(self):
        super().__init__("riskmap_visualizer")

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_bias/risk_map",
            self.on_map,
            10
        )
        # Rviz 시각화용 OccupancyGrid (가벼움)
        self.pub = self.create_publisher(
            OccupancyGrid,
            "/future_bias/risk_grid", 
            10
        )
        self.get_logger().info("🗺️ RiskMap Visualizer (OccupancyGrid) Ready")

    def on_map(self, msg):
        # 1. Float 데이터 수신
        data = np.array(msg.data, dtype=np.float32)
        if data.size != GRID_SIZE * GRID_SIZE:
            return
        
        # 2. 정규화 및 int8 변환 (OccupancyGrid는 0~100)
        # 0.0~1.0 범위를 0~100으로 매핑, 그 이상은 100으로 클리핑
        grid_data = (np.clip(data, 0, 1) * 100).astype(np.int8)

        # 3. 메시지 생성
        og = OccupancyGrid()
        og.header.frame_id = "base_link"
        og.header.stamp = self.get_clock().now().to_msg()
        
        og.info.resolution = RESOLUTION
        og.info.width = GRID_SIZE
        og.info.height = GRID_SIZE
        
        # 원점 설정 (중앙 정렬)
        real_width = GRID_SIZE * RESOLUTION
        # OccupancyGrid의 origin은 지도의 '오른쪽 아래(또는 왼쪽 아래)' 구석 좌표
        og.info.origin.position.x = real_width / 2.0 
        og.info.origin.position.y = real_width / 2.0
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        # 4. 데이터 채우기 (빠름)
        og.data = grid_data.tolist()
        
        self.pub.publish(og)

def main(args=None):
    rclpy.init(args=args)
    node = RiskMapVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()