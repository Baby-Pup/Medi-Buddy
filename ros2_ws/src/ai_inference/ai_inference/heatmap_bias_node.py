#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
T_OUT = 10
NUM_SECTORS = 72   # 360/36 = 10도 단위 섹터
ALPHA = 3.0        # risk → weight 변환 강도

# BEV 해상도 (m per cell) – BEV 생성 쪽이 0.1m로 만들었다고 했으니 맞춰 줌
CELL_SIZE = 0.1

# 거리 감쇠 계수 (클수록 먼 장애물 영향이 더 빨리 줄어듦)
DIST_DECAY = 0.4   # 필요하면 나중에 튜닝


class HeatmapBiasNode(Node):
    def __init__(self):
        super().__init__("heatmap_bias_node")

        # ===== 1. angle / center 미리 계산 (속도 최적화) =====
        self.H = GRID_SIZE
        self.W = GRID_SIZE
        self.cy = self.H // 2
        self.cx = self.W // 2

        # 최대 ray 길이 (그리드 반경만큼만 쏨)
        self.max_ray_steps = min(self.H, self.W) // 2  # 한쪽 방향 최대 셀 수

        # 기존 angle_map / valid_mask는 이제 안 써도 되지만 남겨 둬도 무방
        ys, xs = np.indices((self.H, self.W))
        dy = ys - self.cy
        dx = xs - self.cx
        self.angle_map = np.arctan2(dy, dx)  # [-pi, pi]

        dist = np.sqrt(dx ** 2 + dy ** 2)
        self.valid_mask = dist > 1  # grid cell 단위 (≈ 0.1m) - 발밑 10cm 제거

        # ===== 2. Subscribers / Publishers =====
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/future_occupancy",
            self.on_future_occupancy,
            10
        )

        self.pub_risk = self.create_publisher(
            Float32MultiArray,
            "/future_bias/risk_map",
            10
        )

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

        # (10, H, W) 로 reshape (batch=1 가정)
        future = data.reshape(T_OUT, self.H, self.W)

        # ===== 3. 중간 미래 구간만 사용 (예: t+4 ~ t+8) =====
        # index 기준: 0=t+1, ..., 9=t+10 → 3~7 = t+4~t+8
        mid_indices = [3, 4, 5, 6, 7]
        mid_frames = future[mid_indices]   # (5, H, W)

        # 가까운 미래에 더 높은 가중치
        weights = np.array([0.3, 0.25, 0.2, 0.15, 0.1], dtype=np.float32)  # 합=1

        # (5,H,W) * (5,) → (H,W)
        risk_map = np.tensordot(weights, mid_frames, axes=([0], [0]))

        # ===== 4. risk_map 퍼블리시 (디버깅/시각화용) =====
        risk_msg = Float32MultiArray()
        risk_msg.data = risk_map.flatten().tolist()
        self.pub_risk.publish(risk_msg)

        # ===== 5. risk_map → 방향(섹터)별 위험도 계산 (Ray Sampling) =====
        sector_risk = self.compute_sector_risk(risk_map)

        # ===== 6. sector_risk → omega_weights 변환 =====
        omega_weights = self.risk_to_weights(sector_risk)

        omega_msg = Float32MultiArray()
        omega_msg.data = omega_weights.astype(np.float32).tolist()
        self.pub_omega.publish(omega_msg)

    def compute_sector_risk(self, risk_map: np.ndarray) -> np.ndarray:
        """
        risk_map: (H,W)

        ✅ Ray sampling 방식:
        - 각 섹터의 '중심 각도' 방향으로 ray를 하나 쏜다.
        - 로봇 중심에서 바깥으로 한 칸씩 나아가며 risk_map을 샘플링.
        - 가까운 거리의 큰 risk일수록 영향이 크게, 먼 위험일수록 exp(-DIST_DECAY * 거리)로 감소.
        - 그 ray 위에서 가장 '위험도가 큰' 값을 해당 섹터의 대표 위험도로 사용.

        결과: sector_vals[i] ∈ [0, 1] 근처
        """
        sector_vals = np.zeros(NUM_SECTORS, dtype=np.float32)

        two_pi = 2.0 * np.pi

        for i in range(NUM_SECTORS):
            # 섹터 중앙 각도 (예: -180+5, -170+5, ...)
            angle_center = -np.pi + (i + 0.5) * (two_pi / NUM_SECTORS)

            ray_max_risk = 0.0

            # 로봇 중심에서 바깥으로 ray를 따라가며 샘플링
            # step=2부터: 중심 주변 0~1셀(0~0.1m)은 발밑이라 무시
            for step in range(2, self.max_ray_steps):
                # grid 좌표 (float)
                x = self.cx + step * np.cos(angle_center)
                y = self.cy + step * np.sin(angle_center)

                ix = int(round(x))
                iy = int(round(y))

                # 그리드 밖으로 나가면 해당 ray 종료
                if ix < 0 or ix >= self.W or iy < 0 or iy >= self.H:
                    break

                base_risk = float(risk_map[iy, ix])
                if base_risk <= 0.0:
                    continue

                # 거리 (m)
                dist_m = step * CELL_SIZE

                # 거리 감쇠 적용: 가까운 위험은 크게, 먼 위험은 작게
                # 예: dist_m=0.2 → exp(-0.08)≈0.92
                #     dist_m=3.0 → exp(-1.2)≈0.30
                #     dist_m=8.0 → exp(-3.2)≈0.04
                atten = float(np.exp(-DIST_DECAY * dist_m))
                eff_risk = base_risk * atten

                if eff_risk > ray_max_risk:
                    ray_max_risk = eff_risk

            sector_vals[i] = ray_max_risk

        return sector_vals

    def risk_to_weights(self, sector_risk: np.ndarray) -> np.ndarray:
        """
        risk 값(클수록 위험) → weight (클수록 샘플을 많이 주고 싶은 정도)
        여기서는 안전한 방향일수록 weight가 크도록 변환:
            weight = exp(-α * risk)
        """
        # 위험할수록 작은 weight
        weights = np.exp(-ALPHA * sector_risk)

        # 전부 0이 되는 것 방지
        max_w = np.max(weights)
        if max_w < 1e-6:
            weights[:] = 1.0
        else:
            # 최대 1로 정규화
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
