import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

DESTINATION_TOPIC = '/destination_list'
CLIENT_NAME_TOPIC = '/client_name'
DATE_TOPIC = '/date'
IMAGE_TOPIC = '/camera/image_raw'
FACE_ENCODED_TOPIC = '/face_encoded'
STATUS_TOPIC = '/robot_status'


class QrCodeSubscriber(Node):
    def __init__(self):
        super().__init__('qr_code_subscriber')

        # Publishers
        self.destination_pub = self.create_publisher(String, DESTINATION_TOPIC, 10)
        self.client_pub = self.create_publisher(String, CLIENT_NAME_TOPIC, 10)
        self.date_pub = self.create_publisher(String, DATE_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # Face encoding state
        self.face_ready = False
        self.face_subscriber = self.create_subscription(
            Bool,
            FACE_ENCODED_TOPIC,
            self.face_encoded_callback,
            10
        )

        # Camera subscriber (inactive at start)
        self.bridge = CvBridge()
        self.image_subscription = None

        # QR Detector
        self.qrd = cv2.QRCodeDetector()
        self.last_published_data = ""
        self.qr_scanned = False

    # Face encoding callback
    def face_encoded_callback(self, msg: Bool):
        self.face_ready = msg.data

        if self.face_ready and self.image_subscription is None:
            self.image_subscription = self.create_subscription(
                Image,
                IMAGE_TOPIC,
                self.image_callback,
                10
            )
            self.get_logger().info("📸 Face encoded — started subscribing to /camera/image_raw")

    # Image callback — QR scanning
    def image_callback(self, msg):
        if self.qr_scanned:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        try:
            data, box, _ = self.qrd.detectAndDecode(frame)
        except Exception as e:
            self.get_logger().error(f"detectAndDecode() error: {e}")
            return

        if data:
            date_str, client_name, dest_string = self._parse_qr_data(data)

            if not date_str or not client_name or not dest_string:
                return

            # Prevent duplicate publish
            if dest_string == self.last_published_data:
                return

            self.last_published_data = dest_string
            self.publish_all(date_str, client_name, dest_string)

            # Stop QR scanning
            self.qr_scanned = True
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
            self.get_logger().info("🛑 QR scan complete — stopped subscribing to camera")

    # --------------------------------------------------------
    # QR DATA PARSER
    # --------------------------------------------------------
    def _parse_qr_data(self, raw_data: str):
        # Normalize text
        lines = [line.strip() for line in raw_data.split("\n") if line.strip()]
        lower_lines = [line.strip().lower() for line in lines]

        date_str = ""
        client_name = ""
        destination_list = []

        # 1) 날짜 인식 (예: "2025년 11월 28일")
        for line in lines:
            if "년" in line and "월" in line and "일" in line:
                date_str = line.strip()
                break

        # 2) 이름 + "님 진료..." 형식 인식
        for line in lines:
            lower_line = line.lower()
            # 이름 뒤에 "님"이 들어가고 "진료", "접수", "방문" 등이 포함된 줄
            if "님" in lower_line and ("진료" in lower_line or "접수" in lower_line or "방문" in lower_line):
                client_name = line.split("님")[0].strip()
                break

        # 3) 목적지 리스트 인식
        for line in lines:
            lower_line = line.lower()

            # "1.채혈실", "2. x-ray 실" 같은 패턴
            if len(lower_line) > 2 and lower_line[0].isdigit() and lower_line[1] == '.':
                destination = line[2:].strip()
                destination_list.append(destination)

        destination_string = ", ".join(destination_list)

        return date_str, client_name, destination_string

    # --------------------------------------------------------
    # PUBLISH RESULTS
    # --------------------------------------------------------
    def publish_all(self, date_str, client_name, destination_string):
        # 날짜 퍼블리시
        date_msg = String()
        date_msg.data = date_str
        self.date_pub.publish(date_msg)
        self.get_logger().info(f"📅 Published Date: \"{date_str}\"")

        # 이름 퍼블리시
        name_msg = String()
        name_msg.data = client_name
        self.client_pub.publish(name_msg)
        self.get_logger().info(f"📢 Published Client Name: \"{client_name}\"")

        # 목적지 퍼블리시
        dest_msg = String()
        dest_msg.data = destination_string
        self.destination_pub.publish(dest_msg)
        self.get_logger().info(f"📍 Published Destinations: \"{destination_string}\"")

        # robot_status 퍼블리시
        status_msg = String()
        status_msg.data = "qr_complete"
        self.status_pub.publish(status_msg)
        self.get_logger().info("🤖 Published robot_status: \"qr_complete\"")


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
