#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

# === Only these 2 lines changed ===
GRID_SIZE = 128          # 256 → 128
T_OUT = 16               # 10 → 16
# ==================================

NUM_SECTORS = 72
ALPHA = 3.0
CELL_SIZE = 0.1
DIST_DECAY = 0.4


class HeatmapBiasNode(Node):
    def __init__(self):
        super().__init__("heatmap_bias_node")

        self.H = GRID_SIZE
        self.W = GRID_SIZE
        self.cy = self.H // 2
        self.cx = self.W // 2

        self.max_ray_steps = min(self.H, self.W) // 2

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

        self.get_logger().info("🔥 Heatmap Bias Node (128×128, T_OUT=16) Started")


    def on_future_occupancy(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)
        expected = T_OUT * self.H * self.W

        if data.size != expected:
            self.get_logger().warn(
                f"⚠ future_occupancy size mismatch: got {data.size}, expected {expected}"
            )
            return

        future = data.reshape(T_OUT, self.H, self.W)

        # === Only this line changed ===
        mid_indices = [4,5,6,7,8,9,10]    # 기존 로직 유지 + 길이에 맞게 조정
        # ===============================

        mid_frames = future[mid_indices]

        weights = np.array([0.3, 0.25, 0.2, 0.15, 0.1, 0.05, 0.05], dtype=np.float32)

        risk_map = np.tensordot(weights, mid_frames, axes=([0], [0]))

        msg_r = Float32MultiArray()
        msg_r.data = risk_map.flatten().tolist()
        self.pub_risk.publish(msg_r)

        # === compute_sector_risk + risk_to_weights: NO CHANGES ===
        sector_risk = self.compute_sector_risk(risk_map)
        omega = self.risk_to_weights(sector_risk)

        msg_w = Float32MultiArray()
        msg_w.data = omega.astype(np.float32).tolist()
        self.pub_omega.publish(msg_w)


    # === NO CHANGE BELOW ===
    def compute_sector_risk(self, risk_map):
        sector_vals = np.zeros(NUM_SECTORS, dtype=np.float32)
        two_pi = 2.0 * np.pi

        for i in range(NUM_SECTORS):
            angle_center = -np.pi + (i+0.5) * (two_pi / NUM_SECTORS)
            cos_a = np.cos(angle_center)
            sin_a = np.sin(angle_center)

            ray_max = 0.0

            for step in range(2, self.max_ray_steps):
                row = self.cy - step * cos_a
                col = self.cx - step * sin_a

                iy = int(round(row))
                ix = int(round(col))

                if ix < 0 or ix >= self.W or iy < 0 or iy >= self.H:
                    break

                base = risk_map[iy, ix]
                if base <= 0:
                    continue

                dist_m = step * CELL_SIZE
                atten = np.exp(-DIST_DECAY * dist_m)
                eff = float(base * atten)

                if eff > ray_max:
                    ray_max = eff

            sector_vals[i] = ray_max

        return sector_vals


    def risk_to_weights(self, sector_risk):
        weights = np.exp(-ALPHA * sector_risk)
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
