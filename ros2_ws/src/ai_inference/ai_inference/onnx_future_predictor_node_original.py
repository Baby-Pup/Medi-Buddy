#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import time # 시간 측정용

from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, 
                            ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType)

# =========================
#  CONFIGURATION
# =========================
GRID_SIZE = 256
T_IN = 10   
HEF_PATH = "/home/ubuntu/ros2_ws/src/ai_inference/ai_inference/early_fusion.hef" 

class HailoFuturePredictor(Node):
    def __init__(self):
        super().__init__("hailo_future_predictor")
        
        self.pipeline = None
        self.input_name = None
        self.output_name = None

        self.get_logger().info(f"📦 Loading HEF model: {HEF_PATH}")
        
        try:
            self.target = VDevice()
            self.hef = HEF(HEF_PATH)

            self.configure_params = ConfigureParams.create_from_hef(
                self.hef, interface=HailoStreamInterface.PCIe
            )
            
            self.network_groups = self.target.configure(self.hef, self.configure_params)
            self.network_group = self.network_groups[0]
            self.network_group_params = self.network_group.create_params()

            self.input_params = InputVStreamParams.make(
                self.network_group, format_type=FormatType.FLOAT32
            )
            self.output_params = OutputVStreamParams.make(
                self.network_group, format_type=FormatType.FLOAT32
            )

            self.input_vstream_infos = self.hef.get_input_vstream_infos()
            self.output_vstream_infos = self.hef.get_output_vstream_infos()
            
            self.input_name = self.input_vstream_infos[0].name
            self.output_name = self.output_vstream_infos[0].name
            
            self.get_logger().info(f"✔ Model Loaded. Input: {self.input_name}, Output: {self.output_name}")

        except Exception as e:
            self.get_logger().error(f"❌ Init Failed: {e}")
            raise e

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_pose_sequence",
            self.on_bev_sequence,
            10
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/future_occupancy",
            10
        )
        
        self.get_logger().info("🔮 Node Initialized. Waiting for data...")

    def set_pipeline(self, pipeline):
        self.pipeline = pipeline

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def on_bev_sequence(self, msg: Float32MultiArray):
        if self.pipeline is None:
            self.get_logger().warn("Pipeline not ready.")
            return

        # [DEBUG 1] 데이터 수신 확인
        # self.get_logger().info(f"📥 Received Data! Size: {len(msg.data)}")
        
        seq_flat = np.array(msg.data, dtype=np.float32)
        
        try:
            bev_nchw = seq_flat.reshape(T_IN, GRID_SIZE, GRID_SIZE)
            
            # [DEBUG 2] 전처리 시작
            bev_nhwc = np.ascontiguousarray(bev_nchw.transpose(1, 2, 0))
            input_tensor = bev_nhwc[np.newaxis, ...].astype(np.float32)

            # [DEBUG 3] 추론 직전 (여기서 멈추면 Hailo 장치 문제)
            # self.get_logger().info("🚀 Inferencing...")
            start_time = time.time()

            input_data = {self.input_name: input_tensor}
            
            # === 여기가 핵심 병목 구간 ===
            infer_results = self.pipeline.infer(input_data) 
            # ==========================

            end_time = time.time()
            # [DEBUG 4] 추론 완료 (여기까지 오면 성공)
            # self.get_logger().info(f"✅ Done! Time: {(end_time - start_time):.4f}s")
            
            raw_output = infer_results[self.output_name]
            future_prob = self.sigmoid(raw_output)

            future_prob_nchw = future_prob.transpose(0, 3, 1, 2)
            
            out_msg = Float32MultiArray()
            out_msg.data = future_prob_nchw.flatten().tolist()
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"❌ Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HailoFuturePredictor()

    try:
        # [수정] self.get_logger() -> node.get_logger() 로 변경
        node.get_logger().info("🔌 Activating Network Group...")
        
        with node.network_group.activate(node.network_group_params):
            node.get_logger().info("🌊 Opening VStreams...") # [수정]
            
            with InferVStreams(node.network_group, node.input_params, node.output_params) as pipeline:
                
                node.set_pipeline(pipeline)
                node.get_logger().info("✅ ALL READY! Spin Start.") # [수정]
                
                rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Critical Error: {e}") # [수정]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()