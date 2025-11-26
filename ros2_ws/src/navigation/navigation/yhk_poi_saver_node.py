#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os

class PoiSaverNode(Node):
    def __init__(self):
        super().__init__('poi_saver_node')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)
        self.poi_file = os.path.expanduser('~/ros2_ws/src/navigation/config/poi_map.yaml')
        self.get_logger().info(f"📍 POI 저장 노드 시작됨. RViz에서 클릭 시 좌표 저장됨 → {self.poi_file}")

    def point_callback(self, msg):
        poi_name = input("이 좌표의 이름을 입력하세요 (예: toilet, reception): ")
        data = {poi_name: [msg.point.x, msg.point.y, msg.point.z]}
        # 파일이 이미 있으면 기존 내용 불러오기
        if os.path.exists(self.poi_file):
            with open(self.poi_file, 'r') as f:
                existing = yaml.safe_load(f) or {}
        else:
            existing = {}
        existing.update(data)
        with open(self.poi_file, 'w') as f:
            yaml.dump(existing, f)
        self.get_logger().info(f"✅ {poi_name} 저장 완료: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = PoiSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
