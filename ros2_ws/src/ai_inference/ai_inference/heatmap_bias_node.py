#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
T_OUT = 10
NUM_SECTORS = 72   # 360/72 = 5도 단위 섹터 (정밀도 향상)
ALPHA = 3.0        # risk → weight 변환 강도

# BEV 해상도 (m per cell)
CELL_SIZE = 0.1

# 거리 감쇠 계수
DIST_DECAY = 0.4   


class HeatmapBiasNode(Node):
    def __init__(self):
        super().__init__("heatmap_bias_node")

        # ===== 1. Center 미리 계산 =====
        self.H = GRID_SIZE
        self.W = GRID_SIZE
        self.cy = self.H // 2
        self.cx = self.W // 2

        # 최대 ray 길이 (이미지 밖으로 나가지 않게)
        self.max_ray_steps = min(self.H, self.W) // 2 

        # ===== 2. Subscribers / Publishers =====
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_occupancy",
            self.on_future_occupancy,
            10
        )

        # 디버깅용 (Risk Map 시각화)
        self.pub_risk = self.create_publisher(
            Float32MultiArray,
            "/future_bias/risk_map",
            10
        )

        # Planner로 보낼 가중치
        self.pub_omega = self.create_publisher(
            Float32MultiArray,
            "/future_bias/omega_weights",
            10
        )

        self.get_logger().info("🔥 Heatmap Biasing Node (Ray Sampling) Started.")

    def on_future_occupancy(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)

        expected_size = T_OUT * self.H * self.W
        if data.size != expected_size:
            self.get_logger().warn(
                f"⚠ future_occupancy size mismatch: got {data.size}, expected {expected_size}"
            )
            return

        # (10, H, W) 로 reshape
        future = data.reshape(T_OUT, self.H, self.W)

        # ===== 3. 중간 미래 구간만 사용 (t+4 ~ t+8) =====
        # 너무 먼 미래는 불확실하고, 너무 가까운 미래는 이미 늦음
        mid_indices = [3, 4, 5, 6, 7]
        mid_frames = future[mid_indices]   # (5, H, W)

        # 시간 가중치 (가까운 미래일수록 중요)
        weights = np.array([0.3, 0.25, 0.2, 0.15, 0.1], dtype=np.float32)

        # Weighted Sum -> (H, W) Risk Map 생성
        risk_map = np.tensordot(weights, mid_frames, axes=([0], [0]))

        # ===== 4. risk_map 퍼블리시 (RViz 확인용) =====
        risk_msg = Float32MultiArray()
        risk_msg.data = risk_map.flatten().tolist()
        self.pub_risk.publish(risk_msg)

        # ===== 5. 방향별 위험도 계산 (Ray Sampling) =====
        sector_risk = self.compute_sector_risk(risk_map)

        # ===== 6. omega_weights 변환 및 전송 =====
        omega_weights = self.risk_to_weights(sector_risk)

        omega_msg = Float32MultiArray()
        omega_msg.data = omega_weights.astype(np.float32).tolist()
        self.pub_omega.publish(omega_msg)

    def compute_sector_risk(self, risk_map: np.ndarray) -> np.ndarray:
        """
        [수정됨] bev_creator와 좌표계를 일치시킨 Ray Sampling
        """
        sector_vals = np.zeros(NUM_SECTORS, dtype=np.float32)
        two_pi = 2.0 * np.pi

        for i in range(NUM_SECTORS):
            # 섹터 각도 (-180 ~ +180도)
            # i=0 -> -pi (뒤쪽), i=NUM_SECTORS/2 -> 0 (앞쪽)
            angle_center = -np.pi + (i + 0.5) * (two_pi / NUM_SECTORS)

            ray_max_risk = 0.0

            # Ray Casting
            # bev_creator 로직:
            #   row = cy - (x / res)
            #   col = cx - (y / res)
            # 여기서 x = dist * cos(theta), y = dist * sin(theta)
            
            cos_a = np.cos(angle_center)
            sin_a = np.sin(angle_center)

            for step in range(2, self.max_ray_steps):
                # 🚨 [중요 수정] 좌표계 방향 일치시키기
                # 로봇 앞(0도, cos=1) -> row가 줄어들어야 함 (위로) -> (-) 부호
                # 로봇 왼쪽(90도, sin=1) -> col이 줄어들어야 함 (왼쪽) -> (-) 부호
                
                row = self.cy - step * cos_a  # X축 (Front) 대응
                col = self.cx - step * sin_a  # Y축 (Left) 대응

                iy = int(round(row))
                ix = int(round(col))

                # 맵 밖으로 나가면 종료
                if ix < 0 or ix >= self.W or iy < 0 or iy >= self.H:
                    break

                base_risk = float(risk_map[iy, ix])
                if base_risk <= 0.0:
                    continue

                # 거리 감쇠 적용 (멀수록 영향력 감소)
                dist_m = step * CELL_SIZE
                atten = float(np.exp(-DIST_DECAY * dist_m))
                
                eff_risk = base_risk * atten

                if eff_risk > ray_max_risk:
                    ray_max_risk = eff_risk

            sector_vals[i] = ray_max_risk

        return sector_vals

    def risk_to_weights(self, sector_risk: np.ndarray) -> np.ndarray:
        """
        Risk가 높으면 Weight를 낮춰서 그쪽으로 못 가게 함
        """
        weights = np.exp(-ALPHA * sector_risk)

        # 정규화 (최대값 1.0)
        max_w = np.max(weights)
        if max_w < 1e-6:
            weights[:] = 1.0
        else:
            weights /= max_w

        return weights


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapBiasNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()