import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import time
from medi_buddy_facedetection.modules.face_detection import FaceDetection 

IMAGE_TOPIC = '/camera/image_raw'
ENCODING_DURATION = 3.0  

class DetSubscriber(Node):
    def __init__(self, target_name: str, headless: bool):
        super().__init__('detection_subscriber')
        
        # 얼굴 감지 상태
        self.status_publisher = self.create_publisher(Bool, '/face_detection_status', 10)

        # 얼굴 인코딩 완료 상태 토픽
        self.encoded_publisher = self.create_publisher(Bool, '/face_encoded', 10)

        # 로봇 상태 메시지 publisher 추가
        self.robot_status_pub = self.create_publisher(String, '/robot_status', 10)

        self.target_name = target_name
        self.headless = headless
        self.bridge = CvBridge()
        
        # FaceDetection 객체
        self.face_detector = FaceDetection(
            name=target_name,
            tolerance=0.35, 
            target_detect=self.on_face_event, 
            headless=headless
        )
        
        self.get_logger().info(f"✅ ROS Face Detector Node Initialized. Target: {target_name}, Headless: {headless}")
        
        # --- 인코딩 상태 ---
        self.is_encoding = True
        self.encoding_start_time = self.get_clock().now().nanoseconds / 1e9
        self.frame_buffer = []
        self.known_face_encoding = None
        # -------------------

        self.get_logger().info(f"ℹ️ Starting {ENCODING_DURATION} second encoding capture via ROS subscription...")
        
        # 이미지 구독 시작
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )
        self.get_logger().info(f"▶️ Subscribing to image topic: {IMAGE_TOPIC}")


    def on_face_event(self, detected: bool, name: str):
        msg = Bool()
        msg.data = detected
        self.status_publisher.publish(msg)
        
        if detected:
            log_text = f"★★★ {name} Detected (True) ★★★"
        else:
            log_text = f"☆☆☆ {name} Disappeared (False) ☆★★★"
        self.get_logger().info(f"📡 PUBLISHED: {log_text}")


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        if self.is_encoding:
            self._handle_encoding(frame)
        else:
            if self.known_face_encoding is not None:
                self.face_detector.process_frame(frame)


    def _handle_encoding(self, frame):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        self.frame_buffer.append(frame)
        
        # ⏳ 3초 경과한 경우 인코딩 시도
        if current_time - self.encoding_start_time >= ENCODING_DURATION:
            self.is_encoding = False
            self.get_logger().info("⏳ Encoding capture finished. Processing frames...")

            if self.frame_buffer:
                last_frame = self.frame_buffer[-1]
                self.known_face_encoding = self.face_detector.encode_from_frame(last_frame)

            self.frame_buffer = []  # 버퍼 초기화

            # 인코딩 실패 → 다시 인코딩 모드로 리트라이
            if self.known_face_encoding is None:
                self.get_logger().warn("⚠️ Face encoding failed. Retrying...")
                self.is_encoding = True
                self.encoding_start_time = current_time  # 타이머 리셋
                return

            # 인코딩 성공 → 감지 모드로 전환
            self.face_detector.known_face = self.known_face_encoding
            self.get_logger().info("✅ Target face encoded successfully. Switching to detection mode.")

            # 인코딩 성공 신호 퍼블리시
            encoded_msg = Bool()
            encoded_msg.data = True
            self.encoded_publisher.publish(encoded_msg)
            self.get_logger().info("📡 PUBLISHED: Face encoded = True")

            status_msg = String()
            status_msg.data = "qr_start"
            self.robot_status_pub.publish(status_msg)
            self.get_logger().info("📡 PUBLISHED: /robot_status = qr_start")


    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    TARGET_PERSON_NAME = 'MyBuddy'
    HEADLESS_MODE = True 
    
    node = DetSubscriber(
        target_name=TARGET_PERSON_NAME,
        headless=HEADLESS_MODE
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
