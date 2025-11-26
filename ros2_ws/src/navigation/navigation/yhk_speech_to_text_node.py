#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import DestinationRequest
import speech_recognition as sr

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher = self.create_publisher(DestinationRequest, '/destination_request', 10)
            # 1번. (음성인식 코드 다르게 만든다고 해도 지금 환경에 적용하려면 만들어줘야 하는 필수 부분.)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.get_logger().info("🎙️ 음성 인식 노드가 실행되었습니다. 말을 걸어보세요!")

        # 타이머로 주기적으로 음성 인식 시도
        self.timer = self.create_timer(5.0, self.listen_once)

    def listen_once(self):
        with self.microphone as source:
            self.get_logger().info("🗣️ 듣는 중...")
            audio = self.recognizer.listen(source, phrase_time_limit=4)
        try:
            text = self.recognizer.recognize_google(audio, language="ko-KR")
            self.get_logger().info(f"✅ 인식된 문장: {text}")

            # 예시: "화장실"이라는 단어가 포함되어 있으면 목적지 설정
            if "화장실" in text:
                dest = "화장실"
            elif "접수" in text or "접수처" in text:
                dest = "접수처"
            else:
                self.get_logger().warn("❓ 인식된 단어에 해당하는 목적지를 찾지 못했습니다.")
                return

            msg = DestinationRequest()
                # 2번.
            msg.destination_name = dest
                # 3번.
            self.publisher.publish(msg)
                # 4번. 딱 이렇게 4개만 들어가주면 됨. destination request에 목적지 입력해서 퍼블리쉬하면 semantic_router node에서 받아다가 좌표로 변환하고, 이후 그 좌표를 nav2 쪽으로 보내서 자율주행 시키는 방식.
                    # 물론 음성 인식용 노드 파일도 setup.py에 추가해서 ros2 run으로 돌리든가, 아니면 아예 launch 파일에 노드 추가해서 한꺼번에 돌리든가 해야 되고.
                    # 추가로 다음과 같다고 함.
                        # speech_recognition 패키지는 Google Web Speech API를 사용하기 때문에 인터넷 연결 필요
                        # 오프라인 환경이라면 VOSK, Whisper, Porcupine 같은 로컬 STT 엔진으로 교체 가능
            self.get_logger().info(f"📡 목적지 요청 발행: {dest}")

        except sr.UnknownValueError:
            self.get_logger().warn("🎧 음성을 이해하지 못했습니다.")
        except sr.RequestError as e:
            self.get_logger().error(f"SpeechRecognition API 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
