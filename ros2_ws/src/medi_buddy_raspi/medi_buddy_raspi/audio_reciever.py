#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import subprocess
import os


class AudioRecieverNode(Node):
    def __init__(self):
        super().__init__("audio_recorder_reciever")

        # Subscribe to Base64 MP3 topic
        self.subscription = self.create_subscription(
            String,
            "tts_audio_wav",  # 실제로는 WAV 아님
            self.callback_tts_audio,
            10
        )

        #음성인식
        # 여기에 트리거 퍼블리셔 생성 - 프론트 음성 끝나면 다음 페이지로 넘기는 용도.
        self.tts_done_pub = self.create_publisher(String, "robot_status", 10)


        self.device = "plughw:2,0"  # USB 오디오 장치
        self.get_logger().info("🎧 Audio Player Node Started (MP3 → WAV → aplay)")

    def callback_tts_audio(self, msg):
        try:
            mp3_path = "/tmp/tts_received.mp3"
            wav_path = "/tmp/tts_received.wav"

            # Base64 → MP3 저장
            mp3_bytes = base64.b64decode(msg.data)
            with open(mp3_path, "wb") as f:
                f.write(mp3_bytes)

            self.get_logger().info(f"💾 MP3 저장됨: {mp3_path}")

            # ffmpeg 이용해 WAV (PCM16)으로 변환
            cmd = [
                "ffmpeg", "-y", "-i", mp3_path,
                "-acodec", "pcm_s16le",
                "-ac", "1",
                "-ar", "16000",
                wav_path
            ]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.get_logger().info(f"🔄 WAV 변환 완료: {wav_path}")

            # WAV 재생
            subprocess.run(
                ["aplay", "-D", self.device, wav_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.get_logger().info("🔊 WAV 재생 완료")

            #음성인식
            # 여기에서 퍼블리쉬
            msg_out = String()
            msg_out.data = "tts_done"
            self.tts_done_pub.publish(msg_out)
            self.get_logger().info("📢 재생 완료 트리거 퍼블리시함")

        except Exception as e:
            self.get_logger().error(f"❌ 재생 중 오류 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AudioRecieverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()