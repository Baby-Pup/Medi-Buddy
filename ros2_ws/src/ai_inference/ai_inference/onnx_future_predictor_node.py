#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time

# 입력: BEV Creator에서 오는 Float32
from std_msgs.msg import Float32MultiArray
# 출력: C++로 보낼 때 사용할 UInt8 (바이너리 전송용)
from std_msgs.msg import UInt8MultiArray

from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams,
    FormatType
)

# ================= CONFIG =================
GRID_SIZE = 128
T_IN = 8    # 입력: 과거 8프레임
T_OUT = 16  # 출력: 미래 16프레임
HEF_PATH = "/home/ubuntu/ros2_ws/src/ai_inference/ai_inference/hailo_early.hef"
# ==========================================

class HailoFuturePredictor(Node):
    def __init__(self):
        super().__init__("hailo_future_predictor")

        # 1. 내부 버퍼 (bev_buffer_node 대체)
        self.buffer = [] 

        # 2. Hailo 초기화
        self.pipeline = None
        self.init_hailo()

        # 3. Subscriber (BEV 수신)
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_frame", 
            self.on_frame,
            10
        )

        # 4. Publisher (예측 결과 전송 - UInt8로 최적화)
        self.pub = self.create_publisher(
            UInt8MultiArray,
            "/future_occupancy",
            10
        )

        self.get_logger().info(f"🔮 Hailo Predictor Ready (Buffer: {T_IN}, Output: UInt8)")

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
            
            self.get_logger().info("✔ Hailo Device Initialized")

        except Exception as e:
            self.get_logger().error(f"Hailo Init Error: {e}")
            raise e

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    def on_frame(self, msg: Float32MultiArray):
        # 1. 데이터 수신 (Float32로 복원)
        frame = np.array(msg.data, dtype=np.float32)
        if frame.size != GRID_SIZE * GRID_SIZE:
            return

        # (128, 128) 변환 후 버퍼에 추가
        frame_2d = frame.reshape(GRID_SIZE, GRID_SIZE)
        self.buffer.append(frame_2d)

        # FIFO 관리
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        # 버퍼가 덜 찼으면 대기
        if len(self.buffer) < T_IN:
            return

        if self.pipeline is None:
            return

        try:
            # 2. 추론 입력 준비 (NHWC)
            # Stack: (8, 128, 128)
            seq_nchw = np.stack(self.buffer, axis=0)
            # Transpose: (128, 128, 8)
            seq_nhwc = seq_nchw.transpose(1, 2, 0)
            
            # Add Batch dim: (1, 128, 128, 8)
            input_tensor = np.ascontiguousarray(
                seq_nhwc[np.newaxis, ...],
                dtype=np.float32
            )

            # 3. Hailo 추론
            input_data = {self.input_name: input_tensor}
            raw = self.pipeline.infer(input_data)[self.output_name]
            # Output shape: (1, 128, 128, 16)

            # 4. 후처리
            prob = self.sigmoid(raw)
            
            # C++ 처리를 위해 (Batch, Time, Height, Width) 순서로 변경
            # (1, 16, 128, 128)
            prob_nchw = prob.transpose(0, 3, 1, 2)

            # =========================================================
            # [핵심 최적화] Float32 -> Bytes(UInt8) 직렬화 전송
            # =========================================================
            out_msg = UInt8MultiArray()
            
            # (1) float32 데이터를 바이트 스트림으로 변환 (메모리 뷰)
            raw_bytes = prob_nchw.astype(np.float32).tobytes()
            
            # (2) np.uint8 배열로 재해석 후 list 변환
            # 이 방식이 Python의 int list 생성 중 가장 빠릅니다.
            out_msg.data = np.frombuffer(raw_bytes, dtype=np.uint8).tolist()
            
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HailoFuturePredictor()

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