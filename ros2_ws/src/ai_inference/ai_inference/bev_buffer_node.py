#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Float32MultiArray, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion # 쿼터니언을 오일러/야(Yaw)로 변환하기 위해 필요

# =========================
#  CONFIGURATION & UTILS
# =========================
GRID_SIZE = 256
T_IN = 10
POSE_DIM = 3 # x, y, yaw

def quaternion_to_yaw(q):
    """
    ROS Quaternion (x, y, z, w) -> Yaw (rotation around z-axis)
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class BevBuffer(Node):
    def __init__(self):
        super().__init__("bev_buffer")

        # 버퍼: [ (Timestamp, BEV Array, Pose Array), ... ]
        self.buffer = []  # BEV와 Pose를 함께 저장
        
        # Odom 버퍼: (Timestamp, Odometry Message) - BEV와의 동기화용
        self.odom_history = [] 
        self.max_odom_history = 100 
        
        # 1. BEV 구독 (기존)
        self.sub_bev = self.create_subscription(
            Float32MultiArray,
            "/bev_frame",
            self.on_bev_frame,
            10
        )

        # 2. Odom 구독 (추가)
        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom",  # 실제 사용하는 Odom 토픽명으로 변경 필요
            self.on_odom,
            10
        )

        # 3. 퍼블리셔 (출력 메시지 구조 변경)
        # 이제 BEV + Pose 시퀀스를 담아 보냅니다.
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_pose_sequence", # 토픽 이름 변경 (데이터 구조 명시)
            10
        )

        self.get_logger().info("🧰 BEV Rolling Buffer Node Started with Odom.")

    def on_odom(self, msg: Odometry):
        """Odom 메시지 수신 시 버퍼에 저장"""
        # (timestamp(sec), msg) 형태로 저장
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((ts, msg))
        
        # 버퍼 크기 관리
        if len(self.odom_history) > self.max_odom_history:
            self.odom_history.pop(0)

    def find_nearest_odom(self, target_ts: float):
        """Binary search 대신 간단한 선형 탐색으로 가장 가까운 Odom 포즈 찾기"""
        if not self.odom_history:
            return None

        # 가장 가까운 Odom 메시지 찾기
        min_diff = float('inf')
        nearest_odom_msg = None
        
        for ts, msg in self.odom_history:
            diff = abs(ts - target_ts)
            if diff < min_diff:
                min_diff = diff
                nearest_odom_msg = msg
            elif ts > target_ts and diff > min_diff:
                # Odom 버퍼가 시간 순서대로 정렬되어 있다고 가정하고,
                # 타임스탬프를 지나쳤는데 차이가 다시 커지면 탐색 종료 (최적화)
                break 

        return nearest_odom_msg


    def on_bev_frame(self, msg: Float32MultiArray):
        # 메시지에 타임스탬프가 필요합니다. Float32MultiArray에 타임스탬프가 없으므로,
        # 외부 노드(bev_creator_node.py)에서 Float32MultiArray 대신 Header를 포함하는
        # 커스텀 메시지를 사용했어야 하지만, 여기서는 임시로 현재 시스템 시간을 사용합니다.
        # *******************************************************************
        # 🚨 경고: Float32MultiArray는 Header(Timestamp)가 없습니다.
        # 실제 환경에서는 BEV 프레임을 발행하는 노드에서 Header를 포함한
        # 커스텀 메시지를 사용해야 정확한 동기화가 가능합니다.
        # 여기서는 임시로 현재 노드의 시간을 사용합니다.
        # *******************************************************************
        ts = self.get_clock().now().nanoseconds * 1e-9
        
        bev = np.array(msg.data, dtype=np.float32)

        if bev.size != GRID_SIZE * GRID_SIZE:
            self.get_logger().warn("⚠ Wrong BEV shape received.")
            return

        # 1. Odom 포즈 찾기
        odom_msg = self.find_nearest_odom(ts)
        if odom_msg is None:
            self.get_logger().warn("Odom message not found for BEV frame. Skipping.")
            return

        # 2. Odom -> Pose (x, y, yaw) 변환
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)
        
        pose_arr = np.array([x, y, yaw], dtype=np.float32) # (3,)

        # 3. 버퍼에 저장: (BEV, Pose)
        self.buffer.append((bev, pose_arr))

        # keep last 10 frames
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        # 4. 10프레임 쌓이지 않았으면 패스
        if len(self.buffer) < T_IN:
            return

        # 5. 시퀀스 발행
        # BEV 시퀀스: (10, 256*256)
        # Pose 시퀀스: (10, 3)
        
        bev_seq = np.stack([item[0] for item in self.buffer], axis=0)
        pose_seq = np.stack([item[1] for item in self.buffer], axis=0)

        # 데이터를 묶어서 flatten
        # 최종 구조: [BEV_1...BEV_10, POSE_1...POSE_10]
        # 크기: (10 * 256*256) + (10 * 3)
        
        out_data = np.concatenate([
            bev_seq.flatten(),
            pose_seq.flatten()
        ])
        
        out_msg = Float32MultiArray()
        out_msg.data = out_data.tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BevBuffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()