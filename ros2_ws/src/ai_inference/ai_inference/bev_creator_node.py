#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
# math 모듈 제거 (numpy가 훨씬 빠름)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


GRID_SIZE = 256
RESOLUTION = 0.1    # m per cell
CENTER = GRID_SIZE // 2
RANGE_MAX = 12.0    # 학습 데이터 생성 때 12.0m로 제한했음

def lidar_to_bev(ranges, angle_min, angle_increment):
    # 0으로 초기화된 맵 생성
    bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    # 1. 유효한 데이터 필터링 (벡터 연산)
    # NaN 제거, 너무 가깝거나 먼 거리 제거
    valid_mask = (ranges > 0.03) & (ranges < RANGE_MAX) & (np.isfinite(ranges))
    
    if not np.any(valid_mask):
        return bev

    # 2. 각도 배열 생성
    # np.arange로 한 번에 생성하여 for loop 제거
    num_points = len(ranges)
    all_angles = angle_min + np.arange(num_points) * angle_increment
    
    # 유효한 포인트만 추출
    r = ranges[valid_mask]
    theta = all_angles[valid_mask]

    # 3. 좌표 변환 (Polar -> Cartesian)
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # 4. Grid 좌표 매핑 (학습 코드와 100% 일치시켜야 함!) 🚨
    # ROS 좌표계: X(Front) -> 이미지 위쪽(-Row), Y(Left) -> 이미지 왼쪽(-Col)
    
    # Row 인덱스 (위아래): Center에서 X만큼 뺌
    rows = np.floor(CENTER - (x / RESOLUTION)).astype(int)
    # Col 인덱스 (좌우): Center에서 Y만큼 뺌
    cols = np.floor(CENTER - (y / RESOLUTION)).astype(int)

    # 5. Grid 범위 벗어나는 것 제거 (Boundary Check)
    mask = (rows >= 0) & (rows < GRID_SIZE) & (cols >= 0) & (cols < GRID_SIZE)
    
    # 6. 맵 채우기 (Fancy Indexing)
    bev[rows[mask], cols[mask]] = 1.0

    return bev


class BevCreator(Node):
    def __init__(self):
        super().__init__("bev_creator")

        # QoS 설정을 명시해주는 것이 좋습니다 (센서 데이터는 보통 BestEffort)
        # 하지만 일단 기본 설정으로 두고, 문제 생기면 QoS 수정
        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw", # 혹은 /scan
            self.on_scan,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_frame",
            10
        )

        self.get_logger().info("📡 Real-time BEV Creator Started (Vectorized).")

    def on_scan(self, msg: LaserScan):
        # 리스트를 numpy 배열로 변환
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        bev = lidar_to_bev(
            ranges,
            msg.angle_min,
            msg.angle_increment
        )

        # flatten하여 전송
        out_msg = Float32MultiArray()
        out_msg.data = bev.flatten().tolist()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BevCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()