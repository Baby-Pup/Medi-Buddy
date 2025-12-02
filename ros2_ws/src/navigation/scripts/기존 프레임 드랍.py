import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os
import sys

# Nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Hailo
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams, 
                            InputVStreamParams, OutputVStreamParams, FormatType)

# ==========================================
# 👇 설정
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
HEF_FILE = os.path.join(current_dir, "po_nn3.hef") # 모델 파일명 확인 필수

RGB_TOPIC_NAME = "/ascamera/camera_publisher/rgb0/image"
DEPTH_TOPIC_NAME = "/ascamera/camera_publisher/depth0/image_raw" 
CMD_VEL_TOPIC = "/cmd_vel"
GOAL_TOPIC = "/goal_pose" # 👈 외부에서 오는 목표를 엿듣는 토픽

# 🛑 안전 설정
STOP_DIST_METER = 0.8   
SAFE_WIDTH_RATIO = 0.4  
# ==========================================

global_frame_rgb = None
global_frame_depth = None
global_detections = None
global_running = True
global_ai_fps = 0.0

# --- 전역 변수 (보디가드용) ---
current_goal_msg = None   # 현재 로봇이 가야 할 목표 (엿들은 것)
is_paused_by_guard = False # 가드가 멈췄는지 여부

# --- 칼만 필터 ---
class LowPassFilter:
    def __init__(self, alpha=0.4):
        self.prev_val = None; self.alpha = alpha 
    def filter(self, val):
        if self.prev_val is None: self.prev_val = val; return val
        return (self.prev_val * self.alpha) + (val * (1.0 - self.alpha))

class KalmanTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], dtype=np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], dtype=np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.prediction = np.zeros((2, 1), dtype=np.float32)
        self.first_detected = False
        self.lpf_x = LowPassFilter(alpha=0.6); self.lpf_dist = LowPassFilter(alpha=0.6)

    def update(self, coord_x, coord_dist):
        smooth_x = self.lpf_x.filter(coord_x)
        smooth_dist = self.lpf_dist.filter(coord_dist)
        measurement = np.array([[np.float32(smooth_x)], [np.float32(smooth_dist)]], dtype=np.float32)
        if not self.first_detected:
            self.kalman.statePre = np.array([[np.float32(smooth_x)], [np.float32(smooth_dist)], [0], [0]], dtype=np.float32)
            self.kalman.statePost = self.kalman.statePre.copy()
            self.first_detected = True
        self.kalman.correct(measurement); self.prediction = self.kalman.predict()
        return int(self.prediction[0][0]), int(self.prediction[1][0])

    def predict_future_pos(self, steps=10):
        vx, vy = self.kalman.statePost[2][0], self.kalman.statePost[3][0]
        if abs(vx) > 50 or abs(vy) > 1000: vx = 0; vy = 0
        return int(self.kalman.statePost[0][0] + vx * steps), int(self.kalman.statePost[1][0] + vy * steps)

# --- ROS 노드 (목표 감지 기능 추가) ---
class GuardNode(Node):
    def __init__(self):
        super().__init__('hailo_safety_guard')
        # 카메라 구독
        self.sub_rgb = self.create_subscription(
            Image, RGB_TOPIC_NAME, self.rgb_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, DEPTH_TOPIC_NAME, self.depth_callback, qos_profile_sensor_data)
        
        # 👂 [핵심] 외부 목표 감지 (Rviz, Semantic Router 등)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self.goal_callback, 10)
        
        # 강제 정지용
        self.pub_cmd = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.bridge = CvBridge()

    def rgb_callback(self, msg):
        global global_frame_rgb
        try: global_frame_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def depth_callback(self, msg):
        global global_frame_depth
        try: global_frame_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except: pass

    # 👂 목표가 들어오면 호출됨
    def goal_callback(self, msg):
        global current_goal_msg, is_paused_by_guard
        print("📥 [수신] 새로운 이동 명령을 감지했습니다!")
        current_goal_msg = msg
        is_paused_by_guard = False # 새로운 명령이니 멈춤 해제

    def force_stop(self):
        stop_msg = Twist(); stop_msg.linear.x = 0.0; stop_msg.angular.z = 0.0
        self.pub_cmd.publish(stop_msg)

# --- AI 쓰레드 ---
def ai_worker_thread(pipeline):
    global global_frame_rgb, global_detections, global_running, global_ai_fps
    frame_count = 0; start_time = time.time()
    print("🧠 AI Guard Thread Started.")
    
    while global_running:
        if global_frame_rgb is None: time.sleep(0.01); continue
        try:
            frame = global_frame_rgb.copy()
            resized = cv2.resize(frame, (640, 640))
            input_data = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            input_data = np.expand_dims(input_data, axis=0)
            input_data = np.ascontiguousarray(input_data)
            
            results = pipeline.infer(input_data)
            raw_output = list(results.values())[0]
            
            # 파싱 로직 (Standard Model 기준)
            person_data = raw_output[0][0]
            dets = np.array(person_data)
            if dets.ndim == 2 and dets.shape[0] == 5 and dets.shape[1] != 5:
                dets = np.transpose(dets)
            
            global_detections = dets
            
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed > 1.0: global_ai_fps = frame_count / elapsed; frame_count = 0; start_time = time.time()
        except: time.sleep(0.05)

# --- 메인 함수 ---
def main(args=None):
    global global_running, global_frame_rgb, global_detections, global_ai_fps
    global current_goal_msg, is_paused_by_guard
    
    if not os.path.exists(HEF_FILE):
        print(f"❌ Error: {HEF_FILE} not found!")
        return

    rclpy.init(args=args)
    navigator = BasicNavigator()
    node = GuardNode()
    tracker = KalmanTracker()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    print("🛡️ Safety Guard Initialized. (상시 대기 모드)")
    cv2.namedWindow("Guard View", cv2.WINDOW_NORMAL)

    # Nav2 준비 (활성화될 때까지 대기만 하고, 명령은 안 내림)
    # navigator.waitUntilNav2Active() 
    # print("✅ Nav2 Connected.")

    try:
        with VDevice() as target:
            hef = HEF(HEF_FILE)
            configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            network_groups = target.configure(hef, configure_params)
            network_group = network_groups[0]
            
            input_params = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
            output_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)
            
            with network_group.activate(network_group_params=None):
                with InferVStreams(network_group, input_params, output_params) as pipeline:
                    ai_thread = threading.Thread(target=ai_worker_thread, args=(pipeline,), daemon=True)
                    ai_thread.start()
                    
                    while rclpy.ok():
                        if global_frame_rgb is None: 
                            time.sleep(0.01); continue

                        display_frame = global_frame_rgb.copy()
                        h_img, w_img, _ = display_frame.shape

                        zone_w = int(w_img * SAFE_WIDTH_RATIO)
                        zone_x = int((w_img - zone_w) / 2)
                        box_color = (0, 255, 0)
                        if is_paused_by_guard: box_color = (0, 0, 255)
                        
                        cv2.rectangle(display_frame, (zone_x, 50), (zone_x + zone_w, h_img - 50), box_color, 2)

                        # --- AI 판단 ---
                        is_danger = False
                        current_best_x = None; current_best_dist = None
                        
                        if global_detections is not None:
                            for detection in global_detections:
                                if len(detection) < 5: continue
                                score = detection[4]
                                if score >= 0.4:
                                    ymin, xmin, ymax, xmax = detection[0:4]
                                    x = int(xmin * w_img); y = int(ymin * h_img)
                                    bw = int((xmax - xmin) * w_img); bh = int((ymax - ymin) * h_img)
                                    cx = x + bw // 2; cy = y + bh 
                                    
                                    real_dist_m = 0.0
                                    if global_frame_depth is not None:
                                        try: real_dist_m = global_frame_depth[min(cy, h_img-1), min(cx, w_img-1)] / 1000.0
                                        except: pass
                                    if real_dist_m <= 0: real_dist_m = 4.0 - ((cy/h_img)*3.5)
                                    if real_dist_m < 0: real_dist_m = 0.0

                                    current_best_x = cx
                                    current_best_dist = int(real_dist_m * 1000)

                                    if 0.2 < real_dist_m < STOP_DIST_METER:
                                        if zone_x < cx < (zone_x + zone_w):
                                            is_danger = True
                                            cv2.putText(display_frame, f"BLOCK! {real_dist_m:.1f}m", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                                    
                                    cv2.rectangle(display_frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                                    break 

                        if current_best_x is not None:
                            try:
                                est_x, est_dist = tracker.update(current_best_x, current_best_dist)
                                fx, fd = tracker.predict_future_pos(steps=10)
                                if 300 < fd < (STOP_DIST_METER * 1000) and zone_x < fx < (zone_x + zone_w):
                                    is_danger = True
                                    cv2.arrowedLine(display_frame, (est_x, h_img-50), (fx, h_img-50), (0, 255, 255), 2)
                            except: pass

                        # 🚨 [상시 안전 가드 로직]
                        
                        # 1. 현재 로봇이 목표를 가지고 움직이고 있는가?
                        if current_goal_msg is not None:
                            
                            # 2. 위험 상황 발생
                            if is_danger:
                                # 아직 멈추지 않았다면 -> 멈춰!
                                if not is_paused_by_guard:
                                    print(f"🛑 [위험 감지] 가드 개입! Nav2 정지 (거리: {real_dist_m:.2f}m)")
                                    navigator.cancelTask() # Nav2 취소
                                    is_paused_by_guard = True
                                
                                # 계속해서 급제동 신호 보냄
                                node.force_stop()
                                cv2.putText(display_frame, "GUARD: STOPPED", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

                            # 3. 안전해짐 (복구)
                            else:
                                if is_paused_by_guard:
                                    print("✅ [안전 확보] 원래 목표로 재출발합니다.")
                                    # 아까 엿들었던 목표로 다시 명령 내림
                                    current_goal_msg.header.stamp = navigator.get_clock().now().to_msg()
                                    navigator.goToPose(current_goal_msg)
                                    
                                    is_paused_by_guard = False 
                                    
                                    cv2.putText(display_frame, "GUARD: RESUMING", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                                    cv2.imshow("Guard View", display_frame)
                                    cv2.waitKey(1)
                                    continue 

                                # 4. 정상 주행 중 도착 확인 (도착했으면 목표 변수 초기화)
                                elif navigator.isTaskComplete():
                                    result = navigator.getResult()
                                    # 진짜 도착했거나, 사용자가 강제 취소한 경우 감시 해제
                                    if result == TaskResult.SUCCEEDED or result == TaskResult.CANCELED:
                                        if not is_paused_by_guard: # 가드가 멈춘게 아닌데 멈췄다면
                                            current_goal_msg = None # 목표 달성, 감시 종료
                                            cv2.putText(display_frame, "IDLE (Goal Reached)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                else:
                                    cv2.putText(display_frame, "MONITORING...", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                        else:
                            # 목표가 없을 때 (대기 중)
                            cv2.putText(display_frame, "IDLE (Waiting for Goal)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

                        # 상태 표시
                        cv2.putText(display_frame, f"FPS: {global_ai_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        cv2.imshow("Guard View", display_frame)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            navigator.cancelTask()
                            node.force_stop()
                            break
                            
    except Exception as e: print(f"\n🔥 {e}")
    finally:
        global_running = False
        rclpy.shutdown()
        ros_thread.join()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()