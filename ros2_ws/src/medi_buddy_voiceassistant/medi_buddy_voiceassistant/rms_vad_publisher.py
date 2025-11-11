import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from medi_buddy_voiceassistant.modules.sst import RMS_VAD

class STTPublisher(Node):
    def __init__(self):
        super().__init__('stt_publisher')
        self.publisher_ = self.create_publisher(String, 'voice_text', 10)
        self.vad = RMS_VAD()
        self.get_logger().info("🎙️ STT Publisher Node Started")

    def run(self):
        while rclpy.ok():
            text = self.vad.run()  # 음성 → 텍스트 변환
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f"🗣️ Published: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = STTPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
