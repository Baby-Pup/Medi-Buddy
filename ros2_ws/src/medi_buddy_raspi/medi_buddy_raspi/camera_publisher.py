import rclpy
from rclpy.node import Node
# 이미지 메시지 타입
from sensor_msgs.msg import Image
# OpenCV 브릿지 (OpenCV Mat 객체를 ROS Image 메시지로 변환)
from cv_bridge import CvBridge
import cv2
import threading
import time

# 카메라 ID 설정 (2번 디바이스 사용)
CAMERA_DEVICE_ID = 2
# 발행할 토픽 이름
IMAGE_TOPIC = '/camera/image_raw'

class CameraNode(Node):
    def __init__(self, camera_id=CAMERA_DEVICE_ID):
        super().__init__('camera_node')
        
        # 1. ROS 2 Publisher 설정
        # sensor_msgs.msg.Image 타입으로 토픽 발행
        self.publisher_ = self.create_publisher(
            Image, 
            IMAGE_TOPIC,
            10
        )
        self.get_logger().info(f'Camera Node initialized. Publishing video stream to {IMAGE_TOPIC}')

        # 2. CvBridge 초기화 (OpenCV <-> ROS 메시지 변환 도구)
        self.bridge = CvBridge()

        self.cap_dev_id = camera_id
        self.is_running = True
        
        # 3. 비디오 캡처 루프를 별도 스레드로 실행
        self.capture_thread = threading.Thread(target=self.start_capture_loop)
        self.capture_thread.start()
        self.get_logger().info("Video capture thread started.")

    def start_capture_loop(self):
        """
        카메라를 열고 프레임을 캡처하여 ROS 2 토픽으로 발행하는 메인 루프.
        """
        cap = cv2.VideoCapture(self.cap_dev_id)
        
        if not cap.isOpened():
            self.get_logger().error(f"Error: Could not open camera {self.cap_dev_id}")
            self.is_running = False
            return

        # 캡처 속도 설정을 위한 타이머 (선택 사항: 30 FPS로 제한)
        # 1.0 / 30.0 = 약 0.0333초
        FRAME_DELAY = 1.0 / 30.0 
        
        while self.is_running and rclpy.ok():
            start_time = time.time()
            
            ret, frame = cap.read()

            if ret:
                # 4. 프레임을 ROS 2 Image 메시지로 변환 및 발행
                try:
                    ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                    ros_image_msg.header.frame_id = 'camera_frame'
                    
                    self.publisher_.publish(ros_image_msg)
                    # self.get_logger().debug(f'Published frame.') # 디버깅용
                except Exception as e:
                    self.get_logger().error(f"Error during image conversion or publishing: {e}")
            else:
                self.get_logger().error("Error: Could not read frame from camera.")
                time.sleep(0.1)

            # FPS 제한을 위한 대기
            elapsed_time = time.time() - start_time
            sleep_time = FRAME_DELAY - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)


        # 5. 루프 종료 시 정리
        cap.release()
        self.get_logger().info("Capture loop finished and camera released.")


    def destroy_node(self):
        """
        노드 종료 시 호출되어 스레드를 안전하게 종료
        """
        self.is_running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # OpenCV 로깅 레벨 설정 (경고 메시지 출력 방지)
    cv2.setLogLevel(1) 
    
    node = CameraNode(camera_id=CAMERA_DEVICE_ID)
    
    try:
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