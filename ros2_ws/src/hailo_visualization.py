#!/usr/bin/env python3
"""
hailo_visualization.py

[기능]
1. 'frames_new'의 lidar.npy를 읽어 BEV 변환 (Early Fusion 전처리)
2. Hailo HEF 모델로 추론 (Fix: Network Group Activation 추가)
3. Risk Map 계산 및 시각화 영상 저장
"""

import os
import numpy as np
import cv2
from glob import glob
from tqdm import tqdm

# Hailo Platform Import
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, 
                            ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType)

# ================= 설정 (환경에 맞게 수정하세요) =================
FRAMES_DIR = "./frames_new"           # 데이터 폴더 경로
HEF_PATH = "./early.hef"              # 변환된 HEF 파일 경로
OUTPUT_VIDEO = "visualization_hailo_riskmap.mp4"
FPS = 10

# 중요: 학습/변환시켰던 해상도와 일치해야 합니다. (256)
GRID_SIZE = 256  
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2

# 중요: t_in을 10으로 새로 만드셨다고 했으므로 10으로 설정
T_IN = 10  
T_OUT = 10 

# Risk Map 가중치 (t+4 ~ t+8)
MID_INDICES = [3, 4, 5, 6, 7]
WEIGHTS = np.array([0.3, 0.25, 0.2, 0.15, 0.1], dtype=np.float32)
# ==============================================================

def sigmoid(x):
    """Logits -> Probability (0~1) 변환"""
    return 1 / (1 + np.exp(-x))

def lidar_to_bev(lidar_ranges, meta):
    """
    Lidar Point Cloud -> BEV Image 변환
    """
    bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    
    angle_min, angle_max, angle_inc, range_min, range_max = meta
    max_dist = 12.0 

    valid_mask = (lidar_ranges > 0.03) & (lidar_ranges < max_dist) & (np.isfinite(lidar_ranges))
    if not np.any(valid_mask): return bev

    num_points = len(lidar_ranges)
    angles = angle_min + np.arange(num_points) * angle_inc
    
    r = lidar_ranges[valid_mask]
    theta = angles[valid_mask]

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # ROS 좌표계 기준 (X:Front -> Image Row 위쪽)
    rows = np.floor(CENTER - (x / RESOLUTION)).astype(np.int32)
    cols = np.floor(CENTER - (y / RESOLUTION)).astype(np.int32)

    mask = (rows >= 0) & (rows < GRID_SIZE) & (cols >= 0) & (cols < GRID_SIZE)
    bev[rows[mask], cols[mask]] = 1.0
    
    return bev

def main():
    # 1. Hailo 초기화 및 HEF 로드
    if not os.path.exists(HEF_PATH):
        print(f"[ERROR] HEF file not found: {HEF_PATH}")
        return

    print(f"[INFO] Loading HEF: {HEF_PATH}")
    target = VDevice()
    hef = HEF(HEF_PATH)

    # 네트워크 그룹 설정
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = target.configure(hef, configure_params)[0]
    network_group_params = network_group.create_params()

    # 입출력 파라미터 명시적 생성 (FormatType.FLOAT32 사용)
    input_params = InputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)
    output_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

    # 이름 조회를 위한 Info 객체
    input_vstream_infos = hef.get_input_vstream_infos()
    output_vstream_infos = hef.get_output_vstream_infos()

    # 2. 프레임 데이터 확인
    all_dirs = sorted(glob(os.path.join(FRAMES_DIR, "*")))
    valid_frames = []
    print("[INFO] Checking frames...")
    for d in all_dirs:
        if os.path.exists(os.path.join(d, "lidar.npy")):
            valid_frames.append(d)
    
    n_frames = len(valid_frames)
    if n_frames < T_IN:
        print("[ERROR] Not enough frames.")
        return

    # 3. Video Writer 설정
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # 가로 3배 (Input | Pred | RiskMap)
    out_vid = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, FPS, (GRID_SIZE * 3, GRID_SIZE))
    
    input_buffer = []

    print(f"[INFO] Starting Hailo Inference -> {OUTPUT_VIDEO}")

    # [핵심 수정] activate() 호출
    with network_group.activate(network_group_params):
        with InferVStreams(network_group, input_params, output_params) as infer_pipeline:
            
            for i in tqdm(range(n_frames)):
                # --- 1) 데이터 로드 및 BEV 변환 ---
                frame_path = valid_frames[i]
                lidar = np.load(os.path.join(frame_path, "lidar.npy"))
                meta = np.load(os.path.join(frame_path, "lidar_meta.npy"))
                
                bev = lidar_to_bev(lidar, meta) # (H, W)
                input_buffer.append(bev)

                if len(input_buffer) > T_IN:
                    input_buffer.pop(0)

                # 버퍼가 찰 때까지 대기
                if len(input_buffer) < T_IN:
                    continue

                # --- 2) 입력 데이터 준비 (NHWC 변환) ---
                # Stack: (10, H, W) -> Transpose to (H, W, 10) -> Add Batch (1, H, W, 10)
                input_seq = np.stack(input_buffer, axis=0) 
                input_nhwc = input_seq.transpose(1, 2, 0) 
                input_nhwc = input_nhwc[np.newaxis, ...].astype(np.float32)

                # --- 3) Hailo 추론 실행 ---
                input_data = {input_vstream_infos[0].name: input_nhwc}
                infer_results = infer_pipeline.infer(input_data)
                
                # --- 4) 후처리 (Sigmoid + Risk Map) ---
                raw_output = infer_results[output_vstream_infos[0].name] # (1, H, W, 10)
                
                # Logits -> Sigmoid -> (Batch, Time, H, W) 순서 복구
                preds_prob = sigmoid(raw_output) 
                future_frames = preds_prob[0].transpose(2, 0, 1) # (10, H, W)

                # Risk Map 계산 (t+4 ~ t+8)
                mid_frames = future_frames[MID_INDICES] 
                risk_map = np.tensordot(WEIGHTS, mid_frames, axes=([0], [0]))

                # --- 5) 시각화 ---
                # A. Input (Current)
                img_curr = (bev * 255).astype(np.uint8)
                img_curr = cv2.cvtColor(img_curr, cv2.COLOR_GRAY2BGR)

                # B. Pred (t+5)
                pred_t5 = future_frames[4]
                img_pred = (pred_t5 * 255).astype(np.uint8)
                img_pred_color = cv2.applyColorMap(img_pred, cv2.COLORMAP_JET)

                # C. Risk Map
                img_risk = (np.clip(risk_map, 0, 1) * 255).astype(np.uint8)
                img_risk_color = cv2.applyColorMap(img_risk, cv2.COLORMAP_TURBO)

                # 텍스트 및 마커
                center_pt = (CENTER, CENTER)
                font = cv2.FONT_HERSHEY_SIMPLEX
                
                # Ego Vehicle 표시
                for img in [img_curr, img_pred_color, img_risk_color]:
                    cv2.circle(img, center_pt, 3, (0,0,255), -1)

                cv2.putText(img_curr, "Input (Hailo)", (5, 15), font, 0.4, (0, 255, 0), 1)
                cv2.putText(img_pred_color, "Pred t+5", (5, 15), font, 0.4, (255, 255, 255), 1)
                cv2.putText(img_risk_color, "Risk Map", (5, 15), font, 0.4, (255, 255, 255), 1)

                combined = np.hstack((img_curr, img_pred_color, img_risk_color))
                out_vid.write(combined)

    out_vid.release()
    print(f"[DONE] Saved to {OUTPUT_VIDEO}")

if __name__ == "__main__":
    main()