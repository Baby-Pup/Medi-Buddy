import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

DESTINATION_TOPIC = '/destination_list'
CLIENT_NAME_TOPIC = '/client_name'
IMAGE_TOPIC = '/camera/image_raw'
FACE_ENCODED_TOPIC = '/face_encoded'
STATUS_TOPIC = '/robot_status'   

class QrCodeSubscriber(Node):
    def __init__(self):
        super().__init__('qr_code_subscriber')
        
        # 목적지 퍼블리셔
        self.destination_pub = self.create_publisher(String, DESTINATION_TOPIC, 10)
        self.client_pub = self.create_publisher(String, CLIENT_NAME_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # 얼굴 인코딩 상태
        self.face_ready = False
        self.face_subscriber = self.create_subscription(
            Bool,
            FACE_ENCODED_TOPIC,
            self.face_encoded_callback,
            10
        )

        # 카메라 구독 핸들 (처음에는 None)
        self.bridge = CvBridge()
        self.image_subscription = None   # <--- 처음엔 구독하지 않음

        # QR Detector
        self.qrd = cv2.QRCodeDetector()
        self.last_published_data = ""
        self.qr_scanned = False 


    # 얼굴 인코딩 완료 콜백
    def face_encoded_callback(self, msg: Bool):
        self.face_ready = msg.data

        # 얼굴 인코딩 완료 → 이미지 구독 시작
        if self.face_ready and self.image_subscription is None:
            self.image_subscription = self.create_subscription(
                Image,
                IMAGE_TOPIC,
                self.image_callback,
                10
            )
            self.get_logger().info("📸 Face encoded — Started subscribing to /camera/image_raw")


    # 이미지 콜백 (QR 읽기)
    def image_callback(self, msg):
        if self.qr_scanned:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        if frame is None:
            self.get_logger().error("Received empty frame (None). Skipping.")
            return

        if not hasattr(frame, "shape"):
            self.get_logger().error("Invalid frame received (no shape).")
            return

        if frame.shape[0] == 0 or frame.shape[1] == 0:
            self.get_logger().error(f"Invalid frame shape: {frame.shape}")
            return

        # QR 디코드
        try:
            data, box, _ = self.qrd.detectAndDecode(frame)
        except Exception as e:
            self.get_logger().error(f"detectAndDecode() error: {e}")
            return

        # data, box, _ = self.qrd.detectAndDecode(frame)

        if data:
            client_name, destination_string = self._parse_qr_data(data)

            if not destination_string or not client_name:
                return

            if destination_string == self.last_published_data:
                return

            self.last_published_data = destination_string
            self.publish_all(client_name, destination_string)

            # QR 스캔 완료 → 이미지 구독 취소
            self.qr_scanned = True
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
            self.get_logger().info("🛑 QR scan complete — Stopped subscribing to /camera/image_raw")


    # QR 데이터 파싱
    def _parse_qr_data(self, raw_data: str):
        lines = [line.strip() for line in raw_data.split('\n') if line.strip()]

        client_name = ""
        destination_list = []

        for line in lines:
            if line.startswith("이름:"):
                client_name = line.replace("이름:", "").strip()
            elif len(line) > 2 and line[0].isdigit() and line[1] == '.':
                destination = line[2:].strip()
                destination_list.append(destination)

        destination_string = ", ".join(destination_list)
        return client_name, destination_string


    # 퍼블리시 함수
    def publish_all(self, client_name, destination_string):

        # 1) 이름 퍼블리시
        name_msg = String()
        name_msg.data = client_name
        self.client_pub.publish(name_msg)
        self.get_logger().info(f'📢 Published Client Name: "{client_name}"')

        # 2) 목적지 퍼블리시
        dest_msg = String()
        dest_msg.data = destination_string
        self.destination_pub.publish(dest_msg)
        self.get_logger().info(f'📢 Published Destinations: "{destination_string}"')

        # 3) robot_status 퍼블리시
        status_msg = String()
        status_msg.data = "qr_complete"
        self.status_pub.publish(status_msg)
        self.get_logger().info('📢 Published robot_status: "qr_complete"')


def main(args=None):
    rclpy.init(args=args)
    cv2.setLogLevel(1)
    node = QrCodeSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
