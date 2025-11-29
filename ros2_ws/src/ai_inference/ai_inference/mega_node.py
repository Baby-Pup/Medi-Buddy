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

# LiDAR front only FOV
THETA_MIN = -np.pi/2   # -90 deg
THETA_MAX = +np.pi/2   # +90 deg

# AI Buffer
T_IN = 8
HEF_PATH = "/home/ubuntu/ros2_ws/src/ai_inference/ai_inference/hailo_early.hef"

# Risk Calculation
NUM_SECTORS = 18        # 10° resolution over 180° front view
ALPHA = 3.0
CELL_SIZE = 0.1
DIST_DECAY = 0.4

MID_START = 4
MID_END = 10            # Use t+4 ~ t+10 mid frames


class MegaInferenceNode(Node):
    def __init__(self):
        super().__init__("mega_inference_node")

        # 1. Internal buffer
        self.buffer = []

        # 2. Hailo init
        self.pipeline = None
        self.init_hailo()

        # 3. Raycast LUT for front 18 sectors
        self.init_raycast_lut()

        # 4. Laser subscriber
        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.on_scan,
            10
        )

        # 5. Publish omega weights
        self.pub_omega = self.create_publisher(
            Float32MultiArray,
            "/future_bias/omega_weights",
            10
        )

        self.get_logger().info("🚀 Mega Node (Front 18 sectors) Started")

    # ==========================================
    # Hailo Init
    # ==========================================
    def init_hailo(self):
        try:
            self.target = VDevice()
            self.hef = HEF(HEF_PATH)
            cfg = ConfigureParams.create_from_hef(
                self.hef,
                interface=HailoStreamInterface.PCIe
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
    # Raycast LUT (Front 180° only / 18 sectors)
    # ==========================================
    def init_raycast_lut(self):
        self.ray_lut = []
        max_ray_steps = GRID_SIZE // 2

        # front FOV: -90° ~ +90°
        fov_min = -np.pi/2
        fov_max = +np.pi/2
        fov = fov_max - fov_min

        for i in range(NUM_SECTORS):
            angle_center = fov_min + (i + 0.5) * (fov / NUM_SECTORS)
            cos_a = np.cos(angle_center)
            sin_a = np.sin(angle_center)

            ys, xs, attens = [], [], []

            for step in range(2, max_ray_steps):
                row = CENTER - step * cos_a
                col = CENTER - step * sin_a
                iy = int(round(row))
                ix = int(round(col))

                if ix < 0 or ix >= GRID_SIZE or iy < 0 or iy >= GRID_SIZE:
                    break

                dist_m = step * CELL_SIZE
                atten = np.exp(-DIST_DECAY * dist_m)

                ys.append(iy)
                xs.append(ix)
                attens.append(atten)

            self.ray_lut.append({
                'y': np.array(ys, dtype=int),
                'x': np.array(xs, dtype=int),
                'atten': np.array(attens, dtype=np.float32)
            })

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    # ==========================================
    # Main Callback
    # ==========================================
    def on_scan(self, msg: LaserScan):
        if self.pipeline is None:
            return

        # 1. LiDAR -> BEV
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        mask = (
            (angles >= THETA_MIN) &
            (angles <= THETA_MAX) &
            (ranges > 0.03) &
            (ranges < RANGE_MAX) &
            np.isfinite(ranges)
        )

        r = ranges[mask]
        th = angles[mask]

        x = r * np.cos(th)
        y = r * np.sin(th)

        rows = np.floor(CENTER - (x / RESOLUTION)).astype(int)
        cols = np.floor(CENTER - (y / RESOLUTION)).astype(int)

        valid_rc = (
            (rows >= 0) & (rows < GRID_SIZE) &
            (cols >= 0) & (cols < GRID_SIZE)
        )

        bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        bev[rows[valid_rc], cols[valid_rc]] = 1.0

        # 2. Buffering
        self.buffer.append(bev)
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)
        if len(self.buffer) < T_IN:
            return

        try:
            # 3. Hailo inference
            seq = np.stack(self.buffer, axis=0).transpose(1, 2, 0)
            input_tensor = np.ascontiguousarray(seq[np.newaxis, ...], dtype=np.float32)

            input_data = {self.input_name: input_tensor}
            raw = self.pipeline.infer(input_data)[self.output_name]
            prob = self.sigmoid(raw)

            future_pred = prob[0].transpose(2, 0, 1)

            # 4. Risk Map (single future frame)
            RISK_FRAME = 10  # t+10
            risk_map = future_pred[RISK_FRAME]

            # 5. Front 18-sector risk
            sector_risks = np.zeros(NUM_SECTORS, dtype=np.float32)

            for i in range(NUM_SECTORS):
                lut = self.ray_lut[i]
                if len(lut['y']) == 0:
                    continue

                values = risk_map[lut['y'], lut['x']]
                mask = values > 0

                if np.any(mask):
                    eff = values[mask] * lut['atten'][mask]
                    sector_risks[i] = np.max(eff)
                else:
                    sector_risks[i] = 0.0

            # 6. Omega weights
            w = np.exp(-ALPHA * sector_risks)
            max_w = np.max(w)
            if max_w < 1e-6:
                w[:] = 1.0
            else:
                w /= max_w

            out_msg = Float32MultiArray()
            out_msg.data = w.tolist()
            self.pub_omega.publish(out_msg)

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
