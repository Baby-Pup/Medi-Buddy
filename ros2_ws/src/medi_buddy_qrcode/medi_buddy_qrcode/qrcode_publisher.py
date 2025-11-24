import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 이미지 메시지 타입 구독을 위한 임포트
from sensor_msgs.msg import Image 
# OpenCV 브릿지 임포트
from cv_bridge import CvBridge
import cv2
import threading
import time

# 💡 수정된 토픽 이름
DESTINATION_TOPIC = '/destination_list' 
# 💡 구독할 이미지 토픽 이름
IMAGE_TOPIC = '/camera/image_raw'

class QrCodeSubscriber(Node): # 클래스 이름을 QrCodeSubscriber로 변경 (선택 사항)
    def __init__(self):
        super().__init__('qr_code_subscriber')
        
        # 1. ROS 2 Publisher 설정 (QR 코드 데이터 발행)
        self.publisher_ = self.create_publisher(
            String, 
            DESTINATION_TOPIC,
            10
        )
        self.get_logger().info(f'QR Code Subscriber Node initialized. Publishing destinations to {DESTINATION_TOPIC}')

        # 2. CvBridge 초기화
        self.bridge = CvBridge()
        
        # 3. ROS 2 Subscriber 설정 (카메라 이미지 구독)
        # /camera/image_raw 토픽으로부터 Image 메시지를 받습니다.
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback, # 💡 콜백 함수 설정
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info(f'Subscribing to image topic: {IMAGE_TOPIC}')

        self.last_published_data = "" # 중복 발행 방지용 변수
        
        # 4. QR 코드 디텍터 초기화
        self.qrd = cv2.QRCodeDetector()
        
        # 스레드 대신 ROS 2 콜백 함수(image_callback)에서 모든 처리를 수행합니다.
        # 따라서 기존의 threading.Thread 관련 코드는 모두 제거됩니다.

    def image_callback(self, msg):
        """
        /camera/image_raw 토픽으로부터 이미지를 수신할 때마다 호출되는 콜백 함수.
        """
        try:
            # 5. ROS Image 메시지를 OpenCV Mat (NumPy 배열)으로 변환
            # 'bgr8' 또는 'rgb8'을 사용합니다. CameraNode의 발행 인코딩에 맞추세요.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        # QR 코드 감지 및 디코딩
        data, box, _ = self.qrd.detectAndDecode(frame)

        if data:
            # 6. ROS 2 토픽 발행
            # 이전 데이터와 다를 때만 발행하여 중복을 방지합니다.
            destination_string = self._process_qr_data(data)
            
            # 6. ROS 2 토픽 발행
            # 이전 데이터와 다를 때만 발행하여 중복을 방지합니다.
            if destination_string and destination_string != self.last_published_data:
                self.publish_qr_data(destination_string)
                self.last_published_data = destination_string
            
            # 시각화 (선택 사항: box 그리기)
            if box is not None:
                try:
                    box = box[0].astype(int)
                    # 시각화를 위한 OpenCV 창은 ROS 2의 Spin 루프와 분리해야 하므로
                    # 간단한 테스트 시에만 사용하고 실제 환경에서는 주석 처리합니다.
                    # cv2.polylines(frame, [box], True, (0, 255, 0), 2)
                    pass 
                except IndexError:
                    pass 

        # 시각화 창 띄우기 (옵션)
        # cv2.imshow('QR Code Reader (ROS Subscriber)', frame)
        # cv2.waitKey(1)
        
        # 참고: 콜백 함수는 빠르게 처리하고 리턴해야 합니다. time.sleep()은 필요 없습니다.

    def _process_qr_data(self, raw_data: str) -> str:
        """
        QR 코드 raw 데이터를 분석하여 목적지 리스트 (쉼표로 구분된 문자열)를 추출합니다.
        
        Args:
            raw_data: QR 코드에서 디코딩된 문자열.
        
        Returns:
            "채혈실, X-ray실, 물리치료실, 수납" 형태의 문자열.
        """
        # 1. 데이터를 줄 단위로 나눕니다.
        lines = raw_data.split('\n')
        
        destination_list = []
        
        # 2. 각 줄을 순회하며 목적지를 추출합니다.
        for line in lines:
            # 줄의 앞뒤 공백을 제거하고, 빈 줄은 건너뜁니다.
            line = line.strip()
            if not line:
                continue
            
            # 3. "1. 목적지" 형태의 패턴을 찾습니다. (숫자. 공백 문자열로 시작하는지 확인)
            if line[0].isdigit() and line[1] == '.':
                # "1. " 또는 "2. " 이후의 문자열만 추출합니다.
                # 예: "1. 채혈실" -> "채혈실"
                destination = line[2:].strip()
                if destination:
                    destination_list.append(destination)
            
        # 4. 추출된 목적지 리스트를 쉼표와 공백으로 구분된 단일 문자열로 변환합니다.
        # 예: ["채혈실", "X-ray실", ...] -> "채혈실, X-ray실, 물리치료실, 수납"
        if destination_list:
            return ", ".join(destination_list)
        else:
            self.get_logger().warn("No valid destination found in QR code data.")
            return ""

    def publish_qr_data(self, data):
        """
        디코딩된 QR 코드 데이터를 ROS 2 토픽으로 발행
        """
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Destination: "{data}"')

    def destroy_node(self):
        """
        노드 종료 시 정리
        """
        # 스레드가 없으므로 join()이 필요 없습니다.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # OpenCV 로깅 레벨 설정
    cv2.setLogLevel(1) 
    
    node = QrCodeSubscriber()
    
    try:
        # spin()이 콜백 함수(image_callback)를 반복적으로 호출하여 이미지를 처리합니다.
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user (Ctrl+C).')
    except Exception as e:
        node.get_logger().error(f'An unexpected error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()