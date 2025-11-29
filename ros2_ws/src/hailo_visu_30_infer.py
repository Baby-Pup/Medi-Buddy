#!/usr/bin/env python3
"""
hailo_visualization_30_infer_async.py

[hailo_platform → hailo_infer async version]
"""

import os
import numpy as np
import cv2
from glob import glob
from tqdm import tqdm

from new_hailo_infer import HailoInfer


# ====================== CONFIG ======================
FRAMES_DIR = "./frames_new"
HEF_PATH = "./early_30_real_calib.hef"
OUTPUT_VIDEO = "visualization_hailo_riskmap_30_async.mp4"
FPS = 10

GRID_SIZE = 256
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2

T_IN = 30
MID_INDICES = [5, 10, 15, 20, 25]
WEIGHTS = np.array([0.3, 0.25, 0.2, 0.15, 0.1], dtype=np.float32)
RISK_THRESHOLD = 0.1
# ====================================================


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def lidar_to_bev(lidar_ranges, meta):
    bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    angle_min, angle_max, angle_inc, range_min, range_max = meta

    max_dist = 12.0
    valid_mask = (lidar_ranges > 0.03) & (lidar_ranges < max_dist) & np.isfinite(lidar_ranges)
    if not np.any(valid_mask):
        return bev

    angles = angle_min + np.arange(len(lidar_ranges)) * angle_inc
    r = lidar_ranges[valid_mask]
    theta = angles[valid_mask]

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    rows = (CENTER - x / RESOLUTION).astype(np.int32)
    cols = (CENTER - y / RESOLUTION).astype(np.int32)

    mask = (rows >= 0) & (rows < GRID_SIZE) & (cols >= 0) & (cols < GRID_SIZE)
    bev[rows[mask], cols[mask]] = 1.0

    return bev



# ===========================================================
#           ★★★ CALLBACK 함수 (결과 처리) ★★★
# ===========================================================

class ResultHandler:
    def __init__(self, video_writer):
        self.video_writer = video_writer
        self.latest_output = None

    # ★ 수정된 콜백 시그니처
    def callback(self, bindings_list=None, completion_info=None, **kwargs):
        """
        Hailo run_async가 completion_info를 keyword argument로 넘겨주므로
        그걸 받아줄 수 있도록 시그니처를 열어둔다.
        """
        # partial 에서 bindings_list를 keyword로 넘겨줌
        if bindings_list is None:
            bindings_list = kwargs.get("bindings_list")
        if bindings_list is None:
            # 정말 없으면 그냥 무시
            return

        binding = bindings_list[0]
        outputs = binding.get_output_buffers()  # dict {layer_name: np.array}
        self.latest_output = outputs


def main():

    # =============== HAILO INFER INIT ===============
    inferer = HailoInfer(
        hef_path=HEF_PATH,
        batch_size=1,
        input_type="FLOAT32",
        output_type="FLOAT32"
    )
    # =================================================

    input_vinfo, output_vinfo = inferer.get_vstream_info()
    input_name = input_vinfo[0].name
    output_name = output_vinfo[0].name    # 예: "net/conv3"

    # Frame list
    frames_dir = sorted(glob(os.path.join(FRAMES_DIR, "*")))
    frames = [d for d in frames_dir if os.path.exists(os.path.join(d, "lidar.npy"))]

    # Video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out_vid = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, FPS, (GRID_SIZE * 3, GRID_SIZE))

    handler = ResultHandler(out_vid)

    input_buffer = []

    print(f"[INFO] Running async inference... -> {OUTPUT_VIDEO}")

    for idx in tqdm(range(len(frames))):

        frame_dir = frames[idx]
        lidar = np.load(os.path.join(frame_dir, "lidar.npy"))
        meta = np.load(os.path.join(frame_dir, "lidar_meta.npy"))
        bev = lidar_to_bev(lidar, meta)

        input_buffer.append(bev)
        if len(input_buffer) > T_IN:
            input_buffer.pop(0)

        if len(input_buffer) < T_IN:
            continue

        # (30, H, W) → (H, W, 30)
        input_seq = np.stack(input_buffer, axis=0)
        input_nhwc = input_seq.transpose(1, 2, 0).astype(np.float32)
        batch = [input_nhwc]      # HailoInfer requires list input

        # === ASYNC INFER START ===
        inferer.run(batch, handler.callback)
        inferer.last_infer_job.wait(10000)
        # =================================================

        if handler.latest_output is None:
            continue

        # raw output
        raw_output = handler.latest_output[output_name]     # shape (H, W, 30)

        preds = sigmoid(raw_output)
        future_frames = preds.transpose(2, 0, 1)            # (30, H, W)

        mid_frames = future_frames[MID_INDICES]
        risk_map = np.tensordot(WEIGHTS, mid_frames, axes=([0], [0]))
        risk_map[risk_map < RISK_THRESHOLD] = 0.0

        # ==== Visualization ====
        img_curr = cv2.cvtColor((bev * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)

        pred_mid = future_frames[14]
        img_pred = cv2.applyColorMap((pred_mid * 255).astype(np.uint8), cv2.COLORMAP_JET)

        img_risk = cv2.applyColorMap((np.clip(risk_map, 0, 1) * 255).astype(np.uint8),
                                     cv2.COLORMAP_TURBO)

        for img in (img_curr, img_pred, img_risk):
            cv2.circle(img, (CENTER, CENTER), 3, (0, 0, 255), -1)

        out_vid.write(np.hstack((img_curr, img_pred, img_risk)))


    out_vid.release()
    inferer.close()
    print(f"[DONE] Saved -> {OUTPUT_VIDEO}")


if __name__ == "__main__":
    main()
