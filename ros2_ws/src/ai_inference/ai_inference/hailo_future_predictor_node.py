#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32MultiArray

# Hailo imports
from hailo_platform import (HEF, Device, VStreamInterface,
                            InferVStreams, ConfigureParams)


GRID_SIZE = 256
T_IN = 10
T_OUT = 10


class HailoFuturePredictor(Node):
    def __init__(self):
        super().__init__("hailo_future_predictor")

        # ======== 1. Hailo device 초기화 ========
        hef_path = "/home/ubuntu/hailo/bev_future_tsm_unet.hef"
        
        self.get_logger().info(f"📦 Loading HEF: {hef_path}")
        self.hef = HEF(hef_path)

        self.device = Device()
        configure_params = ConfigureParams.create_from_hef(self.hef)
        self.network_group = self.device.configure(self.hef, configure_params)

        # Python VStream 생성
        self.vstreams = InferVStreams(
            self.network_group,
            VStreamInterface.PYTHON
        )
        self.get_logger().info("🚀 Hailo Inference Engine READY")

        # ======== 2. Subscriber / Publisher ========
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_sequence",
            self.on_bev_sequence,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/future_occupancy",
            10
        )

        self.get_logger().info("🔮 Hailo Future Predictor Node Started")

    # ======== 3. Inference Callback ========
    def on_bev_sequence(self, msg: Float32MultiArray):
        # (10 × 256 × 256) flatten input
        seq_flat = np.array(msg.data, dtype=np.float32)

        if seq_flat.size != T_IN * GRID_SIZE * GRID_SIZE:
            self.get_logger().warn("⚠ Wrong BEV sequence size received")
            return

        # reshape → (1, 10, 256, 256)
        bev_seq = seq_flat.reshape(1, T_IN, GRID_SIZE, GRID_SIZE)

        # ======== Hailo inference ========
        # Hailo는 input/output 모두 ndarray 사용
        outputs = self.vstreams.infer(bev_seq)

        # outputs는 dict일 수도 있으므로 첫 번째 값 사용
        future_logits = None
        for _, arr in outputs.items():
            future_logits = arr
            break

        # shape = (1, 10, 256, 256)
        future_logits = future_logits.reshape(1, T_OUT, GRID_SIZE, GRID_SIZE)

        # sigmoid 적용 (occupancy probability)
        future_occ = 1 / (1 + np.exp(-future_logits))

        # publish as flatten
        out_msg = Float32MultiArray()
        out_msg.data = future_occ.flatten().tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HailoFuturePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
