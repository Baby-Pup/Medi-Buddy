import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import time
from medi_buddy_facedetection.modules.face_detection import FaceDetection


class DetPublisher(Node):
    def __init__(self, target_name: str, headless: bool):
        super().__init__('detection_publisher')
        
        # 1. ROS 2 Publisher 설정
        self.status_publisher = self.create_publisher(Bool, '/face_detection_status', 10)

        self.target_name = target_name
        
        # 2. FaceDetection 객체 생성 및 콜백 함수 연결
        self.face_detector = FaceDetection(
            name=target_name,
            tolerance=0.35, 
            target_detect=self.on_face_event, # 👈 콜백 함수로 ROS 노드 메서드 연결
            headless=headless
        )
        
        self.get_logger().info(f"✅ ROS Face Detector Node Initialized. Target: {target_name}, Headless: {headless}")
        self.get_logger().info("ℹ️ Starting face encoding process...")

        # 3. 얼굴 인코딩 수행
        # (이 함수는 사용자의 입력이 필요할 수 있으므로, 실행 전 준비가 완료되어야 합니다.)
        self.known_face_encoding = self.face_detector.face_encoding()
        
        if self.known_face_encoding is not None:
            self.get_logger().info("✅ Target face encoded successfully.")
        else:
            self.get_logger().error("❌ Face encoding failed. Exiting node.")
            self.destroy_node()
            # rclpy.shutdown()
            return

        # 4. 얼굴 감지 루프를 별도의 쓰레드에서 시작 (ROS spin과 동시에 실행)
        self.detection_thread = threading.Thread(target=self.start_detection_loop)
        self.detection_thread.start()
        
        self.get_logger().info("▶️ Real-time face detection started in a background thread.")


    def on_face_event(self, detected: bool, name: str):
        """
        FaceDetection 클래스에서 타겟 발견/사라짐 시 호출되는 콜백 함수.
        ROS 2 토픽으로 상태를 발행합니다.
        """
        status_msg = Bool()
        status_msg.data = detected
        self.status_publisher.publish(status_msg)
        
        log_msg = String()
        
        if detected:
            log_text = f"★★★ {name} Detected (True) ★★★"
        else:
            log_text = f"☆☆☆ {name} Disappeared (False) ☆★★★"
            
        
        self.get_logger().info(f"📡 PUBLISHED: {log_text}")


    def start_detection_loop(self):
        """
        FaceDetection 클래스의 무한 루프 함수를 실행합니다.
        """
        self.face_detector.face_detection(self.known_face_encoding)


def main(args=None):
    rclpy.init(args=args)
    
    # --- 설정 변수 ---
    TARGET_PERSON_NAME = 'MyBuddy'
    HEADLESS_MODE = True # GUI가 없는 환경이면 True
    # -----------------
    
    node = DetPublisher(
        target_name=TARGET_PERSON_NAME,
        headless=HEADLESS_MODE
    )
    
    try:
        # ROS 2의 메시지 수신 및 타이머 콜백 등을 처리
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()