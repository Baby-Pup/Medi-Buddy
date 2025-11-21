#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
import cv2 # Warping을 위해 OpenCV 추가
import math
from scipy.spatial.transform import Rotation as R # Yaw 추출을 위해 필요

from std_msgs.msg import Float32MultiArray

# =========================
#  CONFIGURATION
# =========================
GRID_SIZE = 256
T_IN = 10
T_OUT = 10
POSE_DIM = 3 # x, y, yaw

RESOLUTION = 0.1 # m per cell (bev_creator.py와 동일)
CENTER = GRID_SIZE // 2
# =========================

# =========================
#  EGO MOTION WARPING UTILS (make_sequences_future.py 로직)
# =========================

def get_pose(pose_arr):
    """
    Pose Array (x, y, yaw)에서 (x, y, yaw) 추출
    (bev_buffer.py에서 이미 x, y, yaw로 변환했다고 가정)
    """
    return pose_arr

def get_se2(x, y, theta):
    """2D 동차변환 행렬 (SE(2)) 생성"""
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return np.array([
        [cos_t, -sin_t, x],
        [sin_t,  cos_t, y],
        [0,      0,     1]
    ])
    
def get_transform_matrix(source_pose, target_pose):
    """
    Source 프레임 픽셀을 Target 프레임 좌표계로 옮기는 Affine Matrix (2x3) 계산
    """
    src_x, src_y, src_theta = source_pose
    tgt_x, tgt_y, tgt_theta = target_pose
    
    # 1. Pixel to Metric (Source)
    # BEV 픽셀 좌표 (u, v) = (col, row) -> Metric (x, y) 변환 행렬
    T_pix2metric = np.array([
        [-RESOLUTION, 0, CENTER * RESOLUTION], # X = -res * v + C*res
        [0, -RESOLUTION, CENTER * RESOLUTION], # Y = -res * u + C*res
        [0, 0, 1]
    ])
    
    # 2. Metric Source to Metric Target (Rigid Body Transform)
    SE2_src = get_se2(src_x, src_y, src_theta)
    SE2_tgt = get_se2(tgt_x, tgt_y, tgt_theta)
    T_rel = np.linalg.inv(SE2_tgt) @ SE2_src

    # 3. Metric to Pixel (Target)
    T_metric2pix = np.linalg.inv(T_pix2metric)

    # Final Matrix: Pixel_tgt = T_metric2pix @ T_rel @ T_pix2metric @ Pixel_src
    M_total = T_metric2pix @ T_rel @ T_pix2metric
    
    return M_total[:2, :] # 3x3에서 2x3 (Affine)만 추출


# =========================
#  NODE CLASS
# =========================

class OnnxFuturePredictor(Node):
    def __init__(self):
        super().__init__("onnx_future_predictor")

        # ... (ONNX 로드 코드는 동일) ...
        # ======== 1. ONNX 모델 로드 ========
        onnx_path = "/home/ubuntu/ros2_ws/src/ai_inference/new.onnx"
        self.get_logger().info(f"📦 Loading ONNX model: {onnx_path}")

        self.session = ort.InferenceSession(
            onnx_path,
            providers=[
                "CUDAExecutionProvider",
                "CPUExecutionProvider"
            ]
        )

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # ======== 2. Subscriber / Publisher ========
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_pose_sequence", # 📢 토픽 이름 변경 (bev_buffer_node와 맞춤)
            self.on_bev_pose_sequence, # 📢 콜백 함수 이름 변경
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/future_occupancy",
            10
        )

        self.get_logger().info("🔮 ONNX Future Predictor Node Started")


    # ======== 3. Inference Callback (Warping 로직 추가) ========
    def on_bev_pose_sequence(self, msg: Float32MultiArray):
        seq_flat = np.array(msg.data, dtype=np.float32)

        # 1. 데이터 분리
        # BEV 데이터 크기: T_IN * GRID_SIZE * GRID_SIZE
        bev_data_size = T_IN * GRID_SIZE * GRID_SIZE
        
        # Odom/Pose 데이터 크기: T_IN * POSE_DIM (10 * 3)
        pose_data_size = T_IN * POSE_DIM
        
        expected_size = bev_data_size + pose_data_size
        
        if seq_flat.size != expected_size:
            self.get_logger().warn(f"⚠ Wrong sequence size received. Expected {expected_size}, Got {seq_flat.size}")
            return
            
        # 데이터 분리: [BEV_1...BEV_10, POSE_1...POSE_10]
        bev_seq_flat = seq_flat[:bev_data_size]
        pose_seq_flat = seq_flat[bev_data_size:]
        
        # BEV reshape: (10, 256, 256)
        bev_seq = bev_seq_flat.reshape(T_IN, GRID_SIZE, GRID_SIZE)
        # Pose reshape: (10, 3)
        pose_seq = pose_seq_flat.reshape(T_IN, POSE_DIM) 

        # 2. Ego Motion Warping 수행 (핵심)
        # 앵커 포즈(기준): 시퀀스의 마지막 포즈 (t 시점)
        anchor_pose = get_pose(pose_seq[T_IN - 1])
        
        warped_bev_list = []
        for i in range(T_IN):
            current_bev = bev_seq[i]
            current_pose = get_pose(pose_seq[i])
            
            # 현재 프레임은 Warping 불필요 (기준 프레임)
            if i == T_IN - 1:
                warped_bev_list.append(current_bev)
                continue
            
            # Warping Matrix 계산 (Source Pose -> Anchor Pose)
            M = get_transform_matrix(current_pose, anchor_pose)
            
            # Warping 실행 (Nearest Neighbor)
            warped_bev = cv2.warpAffine(
                current_bev, M, (GRID_SIZE, GRID_SIZE), 
                flags=cv2.INTER_NEAREST, 
                borderMode=cv2.BORDER_CONSTANT, 
                borderValue=0
            )
            warped_bev_list.append(warped_bev)
            
        # 3. 모델 입력 텐서 구성
        # Warping된 BEV들을 스택하고 배치 차원 추가 → (1, 10, 256, 256)
        bev_seq_warped = np.stack(warped_bev_list, axis=0)
        bev_input_tensor = bev_seq_warped[np.newaxis, :, :, :] # (1, 10, 256, 256)

        # ======== ONNX inference ========
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: bev_input_tensor}
        )
        
        # ... (이후 결과 처리 코드는 동일) ...

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