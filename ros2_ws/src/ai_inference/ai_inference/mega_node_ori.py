#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

# 입력: 라이다
from sensor_msgs.msg import LaserScan
# 출력: C++ 노드가 받을 가중치 (이것만 남김)
from std_msgs.msg import Float32MultiArray

# Hailo AI
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams,
    FormatType
)

# =========================
# CONFIG (기존 코드 통합)
# =========================
GRID_SIZE = 128
RESOLUTION = 0.1
CENTER = GRID_SIZE // 2
RANGE_MAX = 6.4
THETA_MIN = -np.pi/2
THETA_MAX = +np.pi/2

# AI Buffer
T_IN = 8    # 입력 시퀀스
T_OUT = 16  # 출력 시퀀스
HEF_PATH = "/home/ubuntu/ros2_ws/src/ai_inference/ai_inference/hailo_early.hef"

# Risk Calculation
NUM_SECTORS = 72
ALPHA = 3.0
CELL_SIZE = 0.1
DIST_DECAY = 0.4
MID_START = 4
MID_END = 10  # 4~10 프레임 사용


class MegaInferenceNode(Node):
    def __init__(self):
        super().__init__("mega_inference_node")

        # 1. 내부 변수 초기화
        self.buffer = []
        
        # 2. Hailo AI 초기화
        self.pipeline = None
        self.init_hailo()

        # 3. Raycasting 최적화 (Lookup Table 생성)
        # 매번 sin/cos 계산하지 않고 미리 인덱스를 계산해둡니다 (속도 향상 핵심)
        self.init_raycast_lut()

        # 4. Subscriber (LiDAR)
        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",  # 런치파일 이름에 맞춤
            self.on_scan,
            10
        )

        # 5. Publisher (최종 결과물: Omega Weights)
        # C++ 노드(omega_direction_critic)가 받는 유일한 토픽
        self.pub_omega = self.create_publisher(
            Float32MultiArray,
            "/future_bias/omega_weights",
            10
        )

        self.get_logger().info("🚀 Mega Node Started: [LiDAR -> BEV -> AI -> Omega]")

    def init_hailo(self):
        try:
            self.target = VDevice()
            self.hef = HEF(HEF_PATH)
            cfg = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
            self.network_groups = self.target.configure(self.hef, cfg)
            self.network_group = self.network_groups[0]
            self.network_group_params = self.network_group.create_params()
            self.input_params = InputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
            self.output_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
            self.input_name = self.hef.get_input_vstream_infos()[0].name
            self.output_name = self.hef.get_output_vstream_infos()[0].name
            self.get_logger().info("✔ Hailo Initialized")
        except Exception as e:
            self.get_logger().error(f"Hailo Init Error: {e}")

    def init_raycast_lut(self):
        """
        기존 heatmap_bias_node의 2중 for문을 Python에서 돌리면 느립니다.
        __init__에서 미리 계산된 인덱스(Look Up Table)를 만들어두면
        실행 중에는 단순 배열 조회만 하면 되므로 C++만큼 빨라집니다.
        """
        self.ray_lut = [] # 각 섹터별 (y_idx, x_idx, attenuation) 저장
        
        max_ray_steps = min(GRID_SIZE, GRID_SIZE) // 2
        two_pi = 2.0 * np.pi

        for i in range(NUM_SECTORS):
            angle_center = -np.pi + (i + 0.5) * (two_pi / NUM_SECTORS)
            cos_a = np.cos(angle_center)
            sin_a = np.sin(angle_center)

            ys, xs, attens = [], [], []

            for step in range(2, max_ray_steps):
                # 기존 로직과 동일
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
            
            # numpy 배열로 변환하여 저장 (벡터 연산용)
            self.ray_lut.append({
                'y': np.array(ys, dtype=int),
                'x': np.array(xs, dtype=int),
                'atten': np.array(attens, dtype=np.float32)
            })
        self.get_logger().info("✔ Raycast LUT Pre-computed")

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    # ==========================================================
    # MAIN CALLBACK (순차 실행)
    # ==========================================================
    def on_scan(self, msg: LaserScan):
        if self.pipeline is None:
            return

        # ----------------------------------------
        # 1. LiDAR -> BEV 변환 (NumPy 최적화)
        # ----------------------------------------
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Crop & Filter
        mask = (angles >= THETA_MIN) & (angles <= THETA_MAX) & (ranges > 0.03) & (ranges < RANGE_MAX) & np.isfinite(ranges)
        r = ranges[mask]
        th = angles[mask]

        # Polar -> Cartesian -> Grid
        x = r * np.cos(th)
        y = r * np.sin(th)
        rows = np.floor(CENTER - (x / RESOLUTION)).astype(int)
        cols = np.floor(CENTER - (y / RESOLUTION)).astype(int)

        # Boundary Check
        valid_rc = (rows >= 0) & (rows < GRID_SIZE) & (cols >= 0) & (cols < GRID_SIZE)
        
        # BEV 생성
        bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        bev[rows[valid_rc], cols[valid_rc]] = 1.0

        # ----------------------------------------
        # 2. Buffering (8프레임 모으기)
        # ----------------------------------------
        self.buffer.append(bev)
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)
        
        if len(self.buffer) < T_IN:
            return  # 아직 데이터 부족

        # ----------------------------------------
        # 3. AI Inference (Hailo)
        # ----------------------------------------
        # (8, 128, 128) -> (1, 128, 128, 8) NHWC 포맷
        seq = np.stack(self.buffer, axis=0).transpose(1, 2, 0)
        input_tensor = np.ascontiguousarray(seq[np.newaxis, ...], dtype=np.float32)

        # 추론 실행
        input_data = {self.input_name: input_tensor}
        raw = self.pipeline.infer(input_data)[self.output_name]
        # Output: (1, 128, 128, 16)

        # 후처리: Sigmoid -> (16, 128, 128) 로 변환
        prob = self.sigmoid(raw) 
        # (1, 128, 128, 16) -> (16, 128, 128)
        # [0] 꺼내고 -> (128, 128, 16) -> transpose(2, 0, 1) -> (16, 128, 128)
        future_pred = prob[0].transpose(2, 0, 1)

        # ----------------------------------------
        # 4. Risk Map 계산 (numpy.tensordot 활용)
        # ----------------------------------------
        # 중간 프레임(4~10) 추출
        mid_frames = future_pred[MID_START : MID_END+1] # (7, 128, 128)
        weights = np.array([0.3, 0.25, 0.2, 0.15, 0.1, 0.05, 0.05], dtype=np.float32)
        
        # 가중합 계산 (Risk Map 생성)
        risk_map = np.tensordot(weights, mid_frames, axes=([0], [0])) # (128, 128)

        # ----------------------------------------
        # 5. Sector Risk & Omega Weights (LUT 활용 고속 계산)
        # ----------------------------------------
        sector_risks = np.zeros(NUM_SECTORS, dtype=np.float32)

        # Python for문이지만 내부는 numpy 벡터 연산이라 빠름
        for i in range(NUM_SECTORS):
            lut = self.ray_lut[i]
            if len(lut['y']) == 0:
                continue
            
            # 해당 섹터 라인에 있는 Risk 값들을 한방에 가져옴
            values = risk_map[lut['y'], lut['x']]
            
            # 0보다 큰 값에 대해 거리 감쇠 적용
            mask = values > 0
            if np.any(mask):
                eff = values[mask] * lut['atten'][mask]
                sector_risks[i] = np.max(eff)
            else:
                sector_risks[i] = 0.0

        # Risk -> Weights 변환 (Exponential Decay)
        w = np.exp(-ALPHA * sector_risks)
        max_w = np.max(w)
        if max_w < 1e-6:
            w[:] = 1.0
        else:
            w /= max_w

        # ----------------------------------------
        # 6. Publish (최종 결과)
        # ----------------------------------------
        out_msg = Float32MultiArray()
        out_msg.data = w.tolist()
        self.pub_omega.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MegaInferenceNode()
    
    try:
        with node.network_group.activate(node.network_group_params):
            with InferVStreams(node.network_group, node.input_params, node.output_params) as pipeline:
                node.pipeline = pipeline
                rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()