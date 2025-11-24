import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
# 이미지 메시지 타입 구독을 위한 임포트
from sensor_msgs.msg import Image
# OpenCV 브릿지 임포트
from cv_bridge import CvBridge 
import time
# FaceDetection 모듈 임포트
from medi_buddy_facedetection.modules.face_detection import FaceDetection 

IMAGE_TOPIC = '/camera/image_raw'
# 인코딩을 위해 프레임을 캡처할 시간 (3초)
ENCODING_DURATION = 3.0  

class DetSubscriber(Node):
    def __init__(self, target_name: str, headless: bool):
        super().__init__('detection_subscriber')
        
        # 1. ROS 2 Publisher 설정 (상태 발행)
        self.status_publisher = self.create_publisher(Bool, '/face_detection_status', 10)
        self.target_name = target_name
        self.headless = headless
        self.bridge = CvBridge()
        
        # 2. FaceDetection 객체 생성
        self.face_detector = FaceDetection(
            name=target_name,
            tolerance=0.35, 
            target_detect=self.on_face_event, 
            headless=headless
        )
        
        self.get_logger().info(f"✅ ROS Face Detector Node Initialized. Target: {target_name}, Headless: {headless}")
        
        # --- 인코딩 관련 변수 ---
        self.is_encoding = True  # 시작 시 인코딩 모드
        self.encoding_start_time = self.get_clock().now().nanoseconds / 1e9
        self.frame_buffer = []  # 인코딩을 위한 프레임 버퍼
        self.known_face_encoding = None
        # ------------------------

        self.get_logger().info(f"ℹ️ Starting {ENCODING_DURATION} second encoding capture via ROS subscription...")
        
        # 3. ROS 2 Subscriber 설정 (이미지 구독 시작)
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback, # 이미지 콜백으로 인코딩 및 감지 모두 처리
            10
        )
        self.get_logger().info(f"▶️ Subscribing to image topic: {IMAGE_TOPIC}")


    def on_face_event(self, detected: bool, name: str):
        """
        FaceDetection 클래스에서 타겟 발견/사라짐 시 호출되는 콜백 함수.
        ROS 2 토픽으로 상태를 발행합니다.
        """
        status_msg = Bool()
        status_msg.data = detected
        self.status_publisher.publish(status_msg)
        
        if detected:
            log_text = f"★★★ {name} Detected (True) ★★★"
        else:
            log_text = f"☆☆☆ {name} Disappeared (False) ☆★★★"
            
        self.get_logger().info(f"📡 PUBLISHED: {log_text}")


    def image_callback(self, msg):
        """
        /camera/image_raw 토픽으로부터 이미지를 수신하여 인코딩 또는 감지 로직을 실행.
        """
        try:
            # ROS Image 메시지를 OpenCV Mat으로 변환 (bgr8 인코딩 사용)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        if self.is_encoding:
            self._handle_encoding(frame)
        else:
            # 인코딩 완료 후 감지 모드
            if self.known_face_encoding is not None:
                self.face_detector.process_frame(frame)


    def _handle_encoding(self, frame):
        """
        인코딩 모드일 때 프레임을 처리하고, 시간이 지나면 인코딩을 완료.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 프레임을 버퍼에 저장
        self.frame_buffer.append(frame)
        
        # 3초 시간이 경과했는지 확인
        if current_time - self.encoding_start_time >= ENCODING_DURATION:
            self.is_encoding = False
            self.get_logger().info("⏳ Encoding capture finished. Processing frames...")

            if self.frame_buffer:
                # 버퍼에서 마지막 프레임을 사용
                last_frame = self.frame_buffer[-1] 
                
                # FaceDetection 클래스의 수정된 인코딩 함수 호출
                self.known_face_encoding = self.face_detector.encode_from_frame(last_frame) 
            
            self.frame_buffer = [] # 버퍼 해제
            
            if self.known_face_encoding is not None:
                self.get_logger().info("✅ Target face encoded successfully. Switching to detection mode.")
                # 인코딩이 완료된 것을 FaceDetection 클래스에 알려주기 위해 설정
                self.face_detector.known_face = self.known_face_encoding 
            else:
                self.get_logger().error("❌ Face encoding failed. No known face set for detection.")
        
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