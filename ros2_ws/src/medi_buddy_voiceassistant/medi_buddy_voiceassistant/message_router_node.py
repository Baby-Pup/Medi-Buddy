#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import DestinationRequest
from medi_buddy_voiceassistant.modules.tts import TTS

class MessageRouterNode(Node):
    def __init__(self):
        super().__init__('message_router_node')

        # ① publisher: 목적지 요청 (Semantic Router 쪽으로 전달)
        self.dest_pub = self.create_publisher(DestinationRequest, '/destination_request', 10)

        # ② subscriber: 외부 텍스트 입력 수신
        self.subscription = self.create_subscription(
            String,
            '/incoming_text',
            self.listener_callback,
            10
        )

        # ③ TTS 모듈 초기화
        self.tts = TTS()

        self.get_logger().info("📡 Message Router Node Started. Waiting for text input...")

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        self.get_logger().info(f"💬 Received message: {text}")

        # ④ 특정 키워드에 따라 목적지 요청 or 음성 출력
        if "화장실" in text:
            dest = "화장실"
            req = DestinationRequest()
            req.destination_name = dest
            self.dest_pub.publish(req)
            self.get_logger().info(f"🚻 목적지 요청 발행: {dest}")

        elif "접수" in text or "접수처" in text:
            dest = "접수처"
            req = DestinationRequest()
            req.destination_name = dest
            self.dest_pub.publish(req)
            self.get_logger().info(f"🧾 목적지 요청 발행: {dest}")

        else:
            self.get_logger().info("🔊 TTS로 읽기 실행")
            self.tts.make_and_play(text)

def main(args=None):
    rclpy.init(args=args)
    node = MessageRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
