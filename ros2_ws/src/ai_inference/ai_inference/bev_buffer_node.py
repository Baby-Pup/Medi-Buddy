#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

# =========================
#  CONFIGURATION
# =========================
GRID_SIZE = 256
T_IN = 10

class BevBuffer(Node):
    def __init__(self):
        super().__init__("bev_buffer")

        # 순수 BEV 이미지만 저장하는 버퍼
        # 구조: [BEV_Array_1, BEV_Array_2, ...]
        self.buffer = []  
        
        # 1. BEV 구독
        self.sub_bev = self.create_subscription(
            Float32MultiArray,
            "/bev_frame",
            self.on_bev_frame,
            10
        )

        # 2. 시퀀스 퍼블리셔
        # 이름은 기존 연결 유지를 위해 "/bev_pose_sequence" 그대로 둡니다.
        # (실제로는 pose 없이 bev만 들어있음)
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_pose_sequence",
            10
        )

        self.get_logger().info("🧰 BEV Rolling Buffer Node Started (Raw Stacking).")


    def on_bev_frame(self, msg: Float32MultiArray):
        """
        BEV 한 장이 들어오면 -> 버퍼에 넣고 -> 10장 찼으면 묶어서 보냄
        오돔(Odom) 동기화 과정 삭제됨.
        """
        
        # 1. 데이터 수신
        bev = np.array(msg.data, dtype=np.float32)

        # 데이터 크기 체크 (혹시 모를 에러 방지)
        if bev.size != GRID_SIZE * GRID_SIZE:
            self.get_logger().warn(f"⚠ Wrong BEV shape received: {bev.size}")
            return

        # 2. 버퍼에 저장
        self.buffer.append(bev)

        # 3. 오래된 데이터 버리기 (FIFO)
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        # 4. 데이터가 10프레임 미만이면 아직 발행 안 함
        if len(self.buffer) < T_IN:
            # (옵션) 진행상황 로그
            # self.get_logger().info(f"Buffering... {len(self.buffer)}/{T_IN}")
            return

        # 5. 시퀀스 묶기 (Stacking)
        # List of (256*256) -> Numpy (10, 256*256)
        bev_seq = np.stack(self.buffer, axis=0)

        # 6. 발행 (Publish)
        # Pose 데이터 없이 BEV만 보냅니다.
        out_msg = Float32MultiArray()
        out_msg.data = bev_seq.flatten().tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BevBuffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()