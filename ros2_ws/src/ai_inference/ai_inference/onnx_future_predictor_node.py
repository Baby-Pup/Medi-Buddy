#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math

from std_msgs.msg import Float32MultiArray

# Hailo infer helper
from hailo_infer import HailoInfer

# =========================
#  CONFIGURATION
# =========================
GRID_SIZE = 256
T_IN = 10
T_OUT = 10
POSE_DIM = 3
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2


# =========================
#  EGO-MOTION WARPING UTILS
# =========================
def get_pose(pose_arr):
    return pose_arr

def get_se2(x, y, theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return np.array([
        [cos_t, -sin_t, x],
        [sin_t,  cos_t, y],
        [0,      0,     1]
    ])

def get_transform_matrix(source_pose, target_pose):
    src_x, src_y, src_theta = source_pose
    tgt_x, tgt_y, tgt_theta = target_pose

    T_pix2metric = np.array([
        [-RESOLUTION, 0, CENTER * RESOLUTION],
        [0, -RESOLUTION, CENTER * RESOLUTION],
        [0, 0, 1]
    ])

    SE2_src = get_se2(src_x, src_y, src_theta)
    SE2_tgt = get_se2(tgt_x, tgt_y, tgt_theta)
    T_rel = np.linalg.inv(SE2_tgt) @ SE2_src

    T_metric2pix = np.linalg.inv(T_pix2metric)

    M_total = T_metric2pix @ T_rel @ T_pix2metric
    return M_total[:2, :]


# =========================
#  HAILO NODE
# =========================
class HailoFuturePredictor(Node):
    def __init__(self):
        super().__init__("hailo_future_predictor")

        # ===============================
        # 1. Hailo HEF Load
        # ===============================
        hef_path = "/home/ubuntu/ros2_ws/src/ai_inference/new.hef"
        self.get_logger().info(f"📦 Loading Hailo HEF: {hef_path}")

        self.hailo = HailoInfer(
            hef_path=hef_path,
            batch_size=1,
            input_type="FLOAT32",
            output_type="FLOAT32"
        )

        # ===============================
        # 2. ROS2 I/O
        # ===============================
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_pose_sequence",
            self.on_bev_pose_sequence,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/future_occupancy",
            10
        )

        self.get_logger().info("🔮 Hailo Future Predictor Node Started")


    # ===============================
    # 3. Main callback
    # ===============================
    def on_bev_pose_sequence(self, msg: Float32MultiArray):
        seq_flat = np.array(msg.data, dtype=np.float32)

        # ------ reshape ------
        bev_data_size = T_IN * GRID_SIZE * GRID_SIZE
        pose_data_size = T_IN * POSE_DIM

        bev_seq = seq_flat[:bev_data_size].reshape(T_IN, GRID_SIZE, GRID_SIZE)
        pose_seq = seq_flat[bev_data_size:].reshape(T_IN, POSE_DIM)

        # ===============================
        #  Ego-motion Warping
        # ===============================
        anchor_pose = get_pose(pose_seq[T_IN - 1])
        warped = []

        for i in range(T_IN):
            cur_bev = bev_seq[i]
            cur_pose = pose_seq[i]

            if i == T_IN - 1:
                warped.append(cur_bev)
                continue

            M = get_transform_matrix(cur_pose, anchor_pose)
            w = cv2.warpAffine(
                cur_bev, M, (GRID_SIZE, GRID_SIZE),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0
            )
            warped.append(w)

        # final shape: (1, T_IN, 256, 256)
        bev_input = np.stack(warped, axis=0)[np.newaxis, :, :, :]

        # ===============================
        #  IMPORTANT: Convert to NHWC
        #  (1, 10, 256, 256) → (1, 256, 256, 10)
        # ===============================
        bev_input_nhwc = np.transpose(bev_input, (0, 2, 3, 1))

        # ===============================
        # 4. Hailo inference (async)
        #    - result will be returned in callback
        # ===============================
        self.hailo.run(
            input_batch=[bev_input_nhwc],
            inference_callback_fn=self.handle_hailo_output
        )


    # ===============================
    # 5. Hailo async callback
    # ===============================
    def handle_hailo_output(self, bindings_list, infer_job):
        # outputs = dict: {output_name: np.ndarray(...)}
        outputs = {}
        for name, outbuf in bindings_list[0].output().get_buffers().items():
            outputs[name] = outbuf

        # Your model has one output → take first
        future_logits = list(outputs.values())[0]  # (1,256,256,T_OUT)

        # Hailo output is NHWC → convert back to NCHW
        logits_nchw = np.transpose(future_logits, (0, 3, 1, 2))  # (1,10,256,256)

        # sigmoid
        future_occ = 1 / (1 + np.exp(-logits_nchw))

        # publish
        msg = Float32MultiArray()
        msg.data = future_occ.flatten().tolist()
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = HailoFuturePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
