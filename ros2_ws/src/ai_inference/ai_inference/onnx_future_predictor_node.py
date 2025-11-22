#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
from std_msgs.msg import Float32MultiArray

# =========================
#  CONFIGURATION
# =========================
GRID_SIZE = 256
T_IN = 10
# T_OUT = 10  # 추론 노드에선 굳이 안 써도 됨
# POSE_DIM = 3 # 삭제됨 (Pose 안 씀)

class OnnxFuturePredictor(Node):
    def __init__(self):
        super().__init__("onnx_future_predictor")

        # ======== 1. ONNX 모델 로드 ========
        # (학습 후 생성된 onnx 파일 경로로 맞춰주세요)
        onnx_path = "/home/ubuntu/ros2_ws/src/ai_inference/new.onnx"
        self.get_logger().info(f"📦 Loading ONNX model: {onnx_path}")

        # Execution Provider 설정
        # 라즈베리파이/PC 환경에 따라 CUDA가 없으면 CPU로 자동 전환됨
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        
        try:
            self.session = ort.InferenceSession(onnx_path, providers=providers)
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX: {e}")
            raise e

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # ======== 2. Subscriber / Publisher ========
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_pose_sequence", # 토픽 이름은 유지 (bev_buffer와 연결)
            self.on_bev_sequence,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/future_occupancy",
            10
        )

        self.get_logger().info("🔮 ONNX Future Predictor Node Started (No Warping).")


    def on_bev_sequence(self, msg: Float32MultiArray):
        """
        Warping 없이 들어온 BEV 시퀀스를 바로 추론
        """
        # 1. 데이터 받기
        seq_flat = np.array(msg.data, dtype=np.float32)

        # 데이터 크기 검증
        expected_size = T_IN * GRID_SIZE * GRID_SIZE
        if seq_flat.size != expected_size:
            self.get_logger().warn(f"⚠ Wrong sequence size. Expected {expected_size}, Got {seq_flat.size}")
            return

        # 2. Reshape & Batch Dimension 추가
        # Flat -> (10, 256, 256) -> (1, 10, 256, 256)
        # Pose 데이터 분리나 Warping 과정이 싹 사라짐
        bev_input = seq_flat.reshape(1, T_IN, GRID_SIZE, GRID_SIZE)

        # 3. ONNX 추론
        try:
            outputs = self.session.run(
                [self.output_name],
                {self.input_name: bev_input}
            )
            
            # outputs[0] shape: (1, 10, 256, 256)
            future_logits = outputs[0]

            # 4. Sigmoid (Logits -> Probability)
            # 0~1 사이 확률값으로 변환
            future_occ = 1 / (1 + np.exp(-future_logits))

            # 5. 결과 발행
            out_msg = Float32MultiArray()
            out_msg.data = future_occ.flatten().tolist()
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OnnxFuturePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()