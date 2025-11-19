#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort

from std_msgs.msg import Float32MultiArray

GRID_SIZE = 256
T_IN = 10
T_OUT = 10


class OnnxFuturePredictor(Node):
    def __init__(self):
        super().__init__("onnx_future_predictor")

        # ======== 1. ONNX 모델 로드 ========
        onnx_path = "/home/ubuntu/models/bev_future_tsm_unet_online.onnx"
        self.get_logger().info(f"📦 Loading ONNX model: {onnx_path}")

        self.session = ort.InferenceSession(
            onnx_path,
            providers=[
                "CUDAExecutionProvider",
                "CPUExecutionProvider"
            ]
        )

        # input/output 이름 자동 추출
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

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

        self.get_logger().info("🔮 ONNX Future Predictor Node Started")

    # ======== 3. Inference Callback ========
    def on_bev_sequence(self, msg: Float32MultiArray):
        seq_flat = np.array(msg.data, dtype=np.float32)

        if seq_flat.size != T_IN * GRID_SIZE * GRID_SIZE:
            self.get_logger().warn("⚠ Wrong BEV sequence size received")
            return

        # reshape → (1, 10, 256, 256)
        bev_seq = seq_flat.reshape(1, T_IN, GRID_SIZE, GRID_SIZE)

        # ======== ONNX inference ========
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: bev_seq}
        )

        # outputs[0] shape: (1, 10, 256, 256)
        future_logits = outputs[0]

        # sigmoid
        future_occ = 1 / (1 + np.exp(-future_logits))

        # publish
        out_msg = Float32MultiArray()
        out_msg.data = future_occ.flatten().tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OnnxFuturePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
