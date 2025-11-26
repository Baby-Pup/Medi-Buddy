#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import NavEvent  # ✅ 우리가 정의했던 메시지
from datetime import datetime

class NavEventListener(Node):
    def __init__(self):
        super().__init__('nav_event_listener')
        self.subscription = self.create_subscription(
            NavEvent,
            '/nav_event',
            self.nav_event_callback,
            10
        )
        self.get_logger().info("🛰️ NavEvent 리스너 노드가 실행되었습니다!")

    def nav_event_callback(self, msg):
        event_time = datetime.fromtimestamp(msg.stamp.sec).strftime('%H:%M:%S')
        self.get_logger().info(
            f"📍 [{event_time}] 이벤트 수신 → 상태: {msg.status} / 목적지: {msg.destination_name}"
        )

        # 추가 기능: 도착 시 메시지 출력
        if msg.status.lower() == "arrived":
            self.get_logger().info("✅ 목적지 도착! 안내 완료!")
        elif msg.status.lower() == "navigating":
            self.get_logger().info("🛣️ 이동 중...")
        elif msg.status.lower() == "failed":
            self.get_logger().warn("⚠️ 경로 탐색 실패!")

def main(args=None):
    rclpy.init(args=args)
    node = NavEventListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
