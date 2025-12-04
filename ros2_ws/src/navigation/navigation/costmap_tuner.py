#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import math

# PointCloud2 생성 유틸리티 (수정 불필요)
def create_cloud_xyz(header, points):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    itemsize = np.dtype(np.float32).itemsize
    data = np.array(points, dtype=np.float32).tobytes()
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * len(points)),
        data=data
    )

class CostmapTunerNode(Node):
    def __init__(self):
        super().__init__('costmap_tuner_node')
        
        # '가짜 장애물' 발행 (YAML에 설정한 토픽)
        self.publisher_ = self.create_publisher(PointCloud2, '/fake_obstacles', 10)
            
        # -----------------------------------------------------------------
        # ⚠️ [사용자 수정 1] "네모 벽 2개" 좌표 (RViz로 8개 찾아야 함)
        # -----------------------------------------------------------------
        
        # --- 네모 1 (A, B, C, D) 좌표 (예시) ---
        A_POINT = (-1.66, -0.44)
        B_POINT = (-0.27, -0.44)
        C_POINT = (-0.27, 0.15)
        D_POINT = (-1.66, 0.15)
        
        # --- 네모 2 (E, F, G, H) 좌표 (예시) ---
        E_POINT = (0.15, -0.44)
        F_POINT = (1.17, -0.44)
        G_POINT = (1.17, 0.15)
        H_POINT = (0.15, 0.15)
        
        # '네모 벽 1'의 4변을 생성
        self.square_1_points = (
            self.generate_line(A_POINT[0], A_POINT[1], B_POINT[0], B_POINT[1], num_points=100) +
            self.generate_line(B_POINT[0], B_POINT[1], C_POINT[0], C_POINT[1], num_points=100) +
            self.generate_line(C_POINT[0], C_POINT[1], D_POINT[0], D_POINT[1], num_points=100) +
            self.generate_line(D_POINT[0], D_POINT[1], A_POINT[0], A_POINT[1], num_points=100)
        )
        
        # '네모 벽 2'의 4변을 생성
        self.square_2_points = (
            self.generate_line(E_POINT[0], E_POINT[1], F_POINT[0], F_POINT[1], num_points=100) +
            self.generate_line(F_POINT[0], F_POINT[1], G_POINT[0], G_POINT[1], num_points=100) +
            self.generate_line(G_POINT[0], G_POINT[1], H_POINT[0], H_POINT[1], num_points=100) +
            self.generate_line(H_POINT[0], H_POINT[1], E_POINT[0], E_POINT[1], num_points=100)
        )
        
        # '빈 벽' 정의
        self.empty_points = [] 
        # -----------------------------------------------------------------

        # 0.5초마다 'timer_callback' 함수 호출
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Costmap Tuner (네모 벽 2개 '고정' 버전) 시작됨.")
        
        # 시작하자마자 '네모 벽' 2개 한 번 발행
        self.publish_walls(self.square_1_points + self.square_2_points)

    # ⚠️ [수정된 부분] "무조건" '네모 2개' 발행
    def timer_callback(self):
        self.get_logger().info("고정된 '네모 벽' 2개를 계속 발행 중...")
        
        # '네모 1' 리스트와 '네모 2' 리스트를 합쳐서 발행
        combined_points = self.square_1_points + self.square_2_points
        self.publish_walls(combined_points)


    def generate_line(self, start_x, start_y, end_x, end_y, num_points):
        # ... (이 함수는 수정 불필요) ...
        points = []
        if num_points <= 0: return points
        x_vals = np.linspace(start_x, end_x, num_points)
        y_vals = np.linspace(start_y, end_y, num_points)
        for i in range(num_points):
            points.append([float(x_vals[i]), float(y_vals[i]), 0.0]) # Z=0
        return points

    # ⚠️ [수정된 부분] "publish_walls" 함수 (이제 'points' 리스트를 직접 받음)
    def publish_walls(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" # 'map' 프레임 기준
        
        pc2_msg = create_cloud_xyz(header, points)
        self.publisher_.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("종료 전 가짜 벽 정리 중...")
        # ⚠️ [수정된 부분] '빈 리스트'를 직접 전달
        node.publish_walls(node.empty_points) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()