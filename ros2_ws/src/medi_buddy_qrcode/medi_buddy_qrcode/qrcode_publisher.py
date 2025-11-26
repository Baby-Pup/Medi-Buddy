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
        self.destination_pub = self.create_publisher(
            String, 
            DESTINATION_TOPIC,
            10
        )
        self.get_logger().info(f'Publishing destination to {DESTINATION_TOPIC}')

        # 이름 퍼블리셔
        self.client_pub = self.create_publisher(
            String,
            CLIENT_NAME_TOPIC,
            10
        )
        self.get_logger().info(f'Publishing client name to {CLIENT_NAME_TOPIC}')

        # robot_status 퍼블리셔
        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10
        )
        self.get_logger().info(f'Robot status publisher initialized: {STATUS_TOPIC}')

        # 얼굴 인코딩 상태
        self.face_ready = False
        self.face_subscriber = self.create_subscription(
            Bool,
            FACE_ENCODED_TOPIC,
            self.face_encoded_callback,
            10
        )

        # 카메라 이미지 구독
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

        # QR Detector
        self.qrd = cv2.QRCodeDetector()
        self.last_published_data = ""


    # 얼굴 인코딩 완료 콜백
    def face_encoded_callback(self, msg: Bool):
        self.face_ready = msg.data


    # 이미지 콜백 (QR 읽기)
    def image_callback(self, msg):
        if not self.face_ready:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        data, box, _ = self.qrd.detectAndDecode(frame)

        if data:
            client_name, destination_string = self._parse_qr_data(data)

            # 아무 데이터도 없으면 skip
            if not destination_string or not client_name:
                return

            # 이전과 동일한 목적지면 skip
            if destination_string == self.last_published_data:
                return

            self.last_published_data = destination_string
            self.publish_all(client_name, destination_string)


    # QR 데이터 파싱 함수
    def _parse_qr_data(self, raw_data: str):
        """
        이름: 채서린
        ----------
        1. 채혈실
        2. X-ray실
        3. 물리치료실
        4. 수납
        """

        lines = [line.strip() for line in raw_data.split('\n') if line.strip()]

        client_name = ""
        destination_list = []

        for line in lines:

            # 이름 추출
            if line.startswith("이름:"):
                client_name = line.replace("이름:", "").strip()

            # 목적지 목록 추출
            elif len(line) > 2 and line[0].isdigit() and line[1] == '.':
                destination = line[2:].strip()
                destination_list.append(destination)

        destination_string = ", ".join(destination_list)

        return client_name, destination_string


    # 데이터 퍼블리시
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

        # 3) 상태 퍼블리시 (qr_complete)
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
