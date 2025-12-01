#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# LiDAR
from sensor_msgs.msg import LaserScan

# Output for C++ critic
from std_msgs.msg import Float32MultiArray

# Hailo
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams,
    FormatType
)

# =========================
# CONFIG
# =========================
GRID_SIZE = 64
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2
RANGE_MAX = 3.2

# LiDAR front FOV (-90 ~ +90 deg)
THETA_MIN = -np.pi/2
THETA_MAX = +np.pi/2

# AI Buffer
T_IN = 8
HEF_PATH = "/home/ubuntu/ros2_ws/src/ai_inference/ai_inference/hailo_early.hef"

# Risk (front only / 18 sectors)
NUM_SECTORS = 18
ALPHA = 3.0
CELL_SIZE = 0.1
DIST_DECAY = 0.4
RISK_FRAME = 10     # t+10 frame

THRESH = 0.0  # 섹터 위험 임계값


class MegaInferenceNode(Node):
    def __init__(self):
        super().__init__("mega_inference_node")

        # 1. Buffer for last 8 BEVs
        self.buffer = []

        # 2. BEV preallocated (1D flat)
        self.bev_flat = np.zeros(GRID_SIZE * GRID_SIZE, dtype=np.float32)
        self.bev = self.bev_flat.reshape((GRID_SIZE, GRID_SIZE))

        # 3. Hailo init
        self.pipeline = None
        self.init_hailo()

        # 4. Sector LUT
        self.init_raycast_lut()

        # 5. ROS Comms
        self.sub = self.create_subscription(
            LaserScan, "/scan_raw", self.on_scan, 10
        )

        self.pub_omega = self.create_publisher(
            Float32MultiArray, "/future_bias/omega_weights", 10
        )

        self.get_logger().info("🚀 Mega Node (64×64 / 18 sectors / LD19 angle-fixed) Started")

    # ==========================================
    # Hailo Init
    # ==========================================
    def init_hailo(self):
        try:
            self.target = VDevice()
            self.hef = HEF(HEF_PATH)
            cfg = ConfigureParams.create_from_hef(
                self.hef, interface=HailoStreamInterface.PCIe
            )

            self.network_groups = self.target.configure(self.hef, cfg)
            self.network_group = self.network_groups[0]
            self.network_group_params = self.network_group.create_params()

            self.input_params = InputVStreamParams.make(
                self.network_group, format_type=FormatType.FLOAT32
            )
            self.output_params = OutputVStreamParams.make(
                self.network_group, format_type=FormatType.FLOAT32
            )

            self.input_name = self.hef.get_input_vstream_infos()[0].name
            self.output_name = self.hef.get_output_vstream_infos()[0].name

            self.get_logger().info("✔ Hailo Initialized")

        except Exception as e:
            self.get_logger().error(f"Hailo Init Error: {e}")

    # ==========================================
    # Sector LUT (front 18 sectors)
    # ==========================================
    def init_raycast_lut(self):
        self.ray_lut = []
        max_ray = GRID_SIZE // 2

        fov_min = -np.pi/2
        fov_max = +np.pi/2
        fov = fov_max - fov_min

        for i in range(NUM_SECTORS):
            angle_center = fov_min + (i + 0.5) * (fov / NUM_SECTORS)
            ca = np.cos(angle_center)
            sa = np.sin(angle_center)

            ys, xs, att = [], [], []

            for step in range(2, max_ray):
                row = CENTER - step * ca
                col = CENTER - step * sa

                iy = int(round(row))
                ix = int(round(col))

                if not (0 <= ix < GRID_SIZE and 0 <= iy < GRID_SIZE):
                    break

                dist = step * CELL_SIZE
                ys.append(iy)
                xs.append(ix)
                att.append(np.exp(-DIST_DECAY * dist))

            self.ray_lut.append({
                'y': np.array(ys, dtype=int),
                'x': np.array(xs, dtype=int),
                'atten': np.array(att, dtype=np.float32)
            })

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    # ==========================================
    # Main callback
    # ==========================================
    def on_scan(self, msg: LaserScan):
        if self.pipeline is None:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        N = len(ranges)

        # ----------------------------------------------------------
        # 1) Create angle array
        # ----------------------------------------------------------
        angles = msg.angle_min + np.arange(N) * msg.angle_increment

        # ✅ 올바른 랩핑: 0rad = 로봇 정면 유지, [-π, π]로만 바꿈
        angles = (angles + np.pi) % (2*np.pi) - np.pi

        cos_table = np.cos(angles)
        sin_table = np.sin(angles)

        # ----------------------------------------------------------
        # 3) mask (front 180°)
        # ----------------------------------------------------------
        mask = (
            (ranges > 0.03) &
            (ranges < RANGE_MAX) &
            (angles >= THETA_MIN) &  # -90°
            (angles <= THETA_MAX) &  # +90°
            np.isfinite(ranges)
        )

        r = ranges[mask]
        cosv = cos_table[mask]
        sinv = sin_table[mask]

        # ----------------------------------------------------------
        # 4) LiDAR → BEV (fast)
        # ----------------------------------------------------------
        x = r * cosv
        y = r * sinv

        rows = np.floor(CENTER - (x / RESOLUTION)).astype(int)
        cols = np.floor(CENTER - (y / RESOLUTION)).astype(int)

        valid = (
            (rows >= 0) & (rows < GRID_SIZE) &
            (cols >= 0) & (cols < GRID_SIZE)
        )

        self.bev_flat.fill(0)
        idx = rows[valid] * GRID_SIZE + cols[valid]
        self.bev_flat[idx] = 1.0

        bev = self.bev  # view reshape

        # ----------------------------------------------------------
        # 5) Sequence buffer
        # ----------------------------------------------------------
        self.buffer.append(bev.copy())
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)
        if len(self.buffer) < T_IN:
            return

        try:
            # ----------------------------------------------------------
            # 6) Hailo inference
            # ----------------------------------------------------------
            seq = np.stack(self.buffer, axis=0).transpose(1, 2, 0)
            input_tensor = np.ascontiguousarray(
                seq[np.newaxis, ...], dtype=np.float32
            )

            raw = self.pipeline.infer({self.input_name: input_tensor})[self.output_name]
            prob = self.sigmoid(raw)
            future_pred = prob[0].transpose(2, 0, 1)

            # ----------------------------------------------------------
            # 7) Risk map (t+10)
            # ----------------------------------------------------------
            risk_map = future_pred[RISK_FRAME]

            # ----------------------------------------------------------
            # 8) Sector risk (only near 40cm region)
            # ----------------------------------------------------------
            sector_risks = np.zeros(NUM_SECTORS, dtype=np.float32)

            NEAR_DIST = 0.40  # 40cm

            for i in range(NUM_SECTORS):
                lut = self.ray_lut[i]
                if len(lut['y']) == 0:
                    continue

                # 각 좌표의 실제 거리
                dy = (lut['y'] - CENTER) * RESOLUTION
                dx = (lut['x'] - CENTER) * RESOLUTION
                dist = np.sqrt(dx*dx + dy*dy)

                near = dist < NEAR_DIST
                if not np.any(near):
                    sector_risks[i] = 0.0
                    continue

                vals = risk_map[lut['y'][near], lut['x'][near]]

                if np.any(vals > 0):
                    r = np.max(vals)
                    sector_risks[i] = r if r >= THRESH else 0.0
                else:
                    sector_risks[i] = 0.0


            # ----------------------------------------------------------
            # 9) Omega weights
            # ----------------------------------------------------------
            w = np.exp(-ALPHA * sector_risks)
            max_w = np.max(w)
            if max_w > 1e-6:
                w /= max_w

            msg_out = Float32MultiArray()
            msg_out.data = w.tolist()
            self.pub_omega.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Mega Loop Error: {e}")


# ==========================================
# Main
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = MegaInferenceNode()

    try:
        with node.network_group.activate(node.network_group_params):
            with InferVStreams(
                node.network_group,
                node.input_params,
                node.output_params
            ) as pipeline:
                node.pipeline = pipeline
                rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
