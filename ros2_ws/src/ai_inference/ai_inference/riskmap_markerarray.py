#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2  # 128

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
        초록(안전) -> 빨강(위험)
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

        # (H, W) = (Row, Col)
        risk = data.reshape(GRID_SIZE, GRID_SIZE)

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        # 성능 최적화: 전체 픽셀을 다 돌면 느리므로 Step을 두거나
        # numpy where로 유효한 인덱스만 추출하는 것이 좋음.
        # 여기서는 가독성을 위해 이중 루프를 유지하되 좌표 계산만 수정함.
        
        for y in range(0, GRID_SIZE, 2): # (Optional) 2칸씩 건너뛰며 그리기 (부하 감소)
            for x in range(0, GRID_SIZE, 2):
                
                v = risk[y, x] # y는 row, x는 col
                
                # 노이즈 제거 (너무 낮은 값은 안 그림)
                if v < 0.1: 
                    continue

                r, g, b = self.color_from_value(v)

                m = Marker()
                m.header.frame_id = "base_link" # 로봇 기준 좌표계
                m.header.stamp = now
                m.id = marker_id
                marker_id += 1

                m.type = Marker.CUBE
                m.action = Marker.ADD
                # 마커 크기
                m.scale.x = RESOLUTION * 2 # 건너뛰었으니 조금 키움
                m.scale.y = RESOLUTION * 2
                m.scale.z = 0.05 # 높이 약간 줌

                # 🚨 [좌표 변환 수정 핵심]
                # bev_creator: row = CENTER - (Real_X / res)
                # 역산: Real_X = (CENTER - row) * res
                # 여기서 row는 y, col은 x임.
                
                real_x = (CENTER - y) * RESOLUTION
                real_y = (CENTER - x) * RESOLUTION

                m.pose.position.x = real_x
                m.pose.position.y = real_y
                m.pose.position.z = 0.0  # 바닥에 깔기

                m.pose.orientation.w = 1.0

                m.color.r = r
                m.color.g = g
                m.color.b = b
                m.color.a = 0.6 # 약간 투명하게

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