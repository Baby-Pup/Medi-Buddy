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
import collections

# Nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Hailo
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams, 
                            InputVStreamParams, OutputVStreamParams, FormatType)

# ==========================================
# 👇 설정 (요청하신 1.5m / 2.0m 적용)
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
HEF_FILE = os.path.join(current_dir, "po_nn3.hef")

RGB_TOPIC_NAME = "/ascamera/camera_publisher/rgb0/image"
DEPTH_TOPIC_NAME = "/ascamera/camera_publisher/depth0/image_raw" 
CMD_VEL_TOPIC = "/cmd_vel"
GOAL_TOPIC = "/goal_pose"

# 🛑 [안전 판단 기준 - 수정됨]
STOP_DIST_METER = 1.5    # 1.5m 이내 진입 시 위험(정지)
RESUME_DIST_METER = 2.0  # 2.0m 이상 멀어져야 안전(재출발)

# 점수 기준 (거리에 따라 점수가 변하므로 이에 맞춰 설정)
RISK_THRESHOLD = 60.0    # 60점 이상이면 정지
RESUME_THRESHOLD = 20.0  # 20점 이하로 뚝 떨어져야 재출발 (안전 마진 확보)

SHOW_GUI = True
GUI_SKIP_RATE = 2 
# ==========================================

global_frame_rgb = None
global_frame_depth = None
global_detections = None
global_running = True
global_ai_fps = 0.0

current_goal_msg = None
is_paused_by_guard = False

# 🧠 [AI 엔진 1] 궤적 예측기
class TrajectoryPredictor:
    def __init__(self, history_len=10):
        self.history = collections.deque(maxlen=history_len)
    
    def update(self, dist_m, x_pixel):
        now = time.time()
        self.history.append([now, dist_m, x_pixel])
        
    def predict_2s_later(self):
        if len(self.history) < 5: return None, None 

        data = np.array(self.history)
        t = data[:, 0] - data[-1, 0]
        d = data[:, 1]
        x = data[:, 2]

        try:
            # 1차 함수(직선) 추세 예측
            coeff_d = np.polyfit(t, d, 1)
            coeff_x = np.polyfit(t, x, 1)
            
            future_t = 2.0 # 2초 뒤
            pred_dist = np.polyval(coeff_d, future_t)
            pred_x = int(np.polyval(coeff_x, future_t))
            
            return pred_dist, pred_x
        except:
            return None, None

# 🧠 [AI 엔진 2] 위험 점수 계산기 (1.5m / 2.0m 로직 반영)
def calculate_risk_score(current_dist, pred_dist_2s, screen_w, current_x, pred_x):
    score = 0.0
    
    # 1. 거리 점수 계산 (2.2m부터 점수 부여 시작)
    # 2.0m 밖이면 점수가 거의 0에 수렴하도록 설정
    max_detect_range = 2.2 
    
    if current_dist < max_detect_range:
        # 거리가 가까울수록 점수 급상승
        # 예: 1.5m -> (2.2 - 1.5) * 90 = 63점 (정지 기준 초과)
        # 예: 2.0m -> (2.2 - 2.0) * 90 = 18점 (재출발 기준 이하)
        score += (max_detect_range - current_dist) * 90
    
    # 2. 접근 점수 (다가오면 가산점)
    if pred_dist_2s is not None:
        if pred_dist_2s < current_dist: 
            approaching_speed = current_dist - pred_dist_2s
            score += approaching_speed * 20 
            
            # 미래에 1.5m 안으로 들어올 것 같으면?
            if pred_dist_2s < STOP_DIST_METER:
                score += 30 # 예측 위험 보너스

    # 3. 정면 일치 점수 (화면 중앙일수록 위험)
    center = screen_w / 2
    target = pred_x if pred_x is not None else current_x
    deviation = abs(target - center) / (screen_w / 2)
    
    if deviation < 0.6: # 중앙 60% 이내
        score += (0.6 - deviation) * 20

    return max(0.0, min(100.0, score))

# --- ROS 노드 ---
class GuardNode(Node):
    def __init__(self):
        super().__init__('hailo_safety_guard')
        self.sub_rgb = self.create_subscription(
            Image, RGB_TOPIC_NAME, self.rgb_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, DEPTH_TOPIC_NAME, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self.goal_callback, 10)
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

    def goal_callback(self, msg):
        global current_goal_msg, is_paused_by_guard
        print(f"\n📥 [수신] AI 가드 활성화 (정지:{STOP_DIST_METER}m / 재개:{RESUME_DIST_METER}m)")
        current_goal_msg = msg
        is_paused_by_guard = False

    def force_stop(self):
        stop_msg = Twist(); stop_msg.linear.x = 0.0; stop_msg.angular.z = 0.0
        self.pub_cmd.publish(stop_msg)

# 멀티 포인트 거리 측정
def get_multi_point_distance(depth_img, x1, y1, x2, y2, h_img, w_img):
    if depth_img is None: return 0.0
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    width = x2 - x1; height = y2 - y1
    points = [
        (cx, cy), (cx, max(0, cy - height//4)), (cx, min(h_img, cy + height//4)),
        (max(0, cx - width//4), cy), (min(w_img, cx + width//4), cy)
    ]
    valid_dists = []
    for px, py in points:
        roi = depth_img[max(0, py-5):min(h_img, py+5), max(0, px-5):min(w_img, px+5)]
        vals = roi[roi > 0]
        if vals.size > 0:
            d = np.median(vals) / 1000.0
            if 0.2 < d < 8.0: valid_dists.append(d)
    
    if not valid_dists: return 0.0
    return min(valid_dists) 

# --- AI 쓰레드 ---
def ai_worker_thread(pipeline):
    global global_frame_rgb, global_detections, global_running, global_ai_fps
    frame_count = 0; start_time = time.time()
    print("🧠 AI Guard Thread Started.")
    
    while global_running:
        if global_frame_rgb is None: time.sleep(0.01); continue
        try:
            if SHOW_GUI: frame = global_frame_rgb.copy()
            else: frame = global_frame_rgb

            resized = cv2.resize(frame, (640, 640))
            input_data = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            input_data = np.expand_dims(input_data, axis=0)
            input_data = np.ascontiguousarray(input_data)
            
            results = pipeline.infer(input_data)
            raw_output = list(results.values())[0]
            
            person_data = raw_output[0][0]
            dets = np.array(person_data)
            if dets.ndim == 2 and dets.shape[0] == 5 and dets.shape[1] != 5:
                dets = np.transpose(dets)
            
            global_detections = dets
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed > 1.0: 
                global_ai_fps = frame_count / elapsed; frame_count = 0; start_time = time.time()
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
    predictor = TrajectoryPredictor()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    print(f"🛡️ Safety Guard Initialized (Stop: {STOP_DIST_METER}m -> Resume: {RESUME_DIST_METER}m).")
    
    if SHOW_GUI:
        cv2.namedWindow("AI Guard View", cv2.WINDOW_NORMAL)

    gui_counter = 0

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
                        if global_frame_rgb is None: time.sleep(0.01); continue

                        h_img, w_img, _ = global_frame_rgb.shape
                        
                        # AI 판단 변수
                        risk_score = 0.0
                        detected_x = None; detected_dist = 0.0
                        pred_x = None; pred_dist = None
                        
                        local_dets = None
                        if global_detections is not None: local_dets = global_detections.copy()

                        # 가장 큰(가까운) 사람 찾기
                        target_person = None
                        max_area = 0

                        if local_dets is not None:
                            for detection in local_dets:
                                if len(detection) < 5: continue
                                score = detection[4]
                                if score < 0.45: continue

                                ymin, xmin, ymax, xmax = detection[0:4]
                                area = (xmax - xmin) * (ymax - ymin)
                                
                                if area > max_area:
                                    max_area = area
                                    target_person = detection

                        # 타겟이 있으면 위험도 계산
                        if target_person is not None:
                            ymin, xmin, ymax, xmax = target_person[0:4]
                            x1 = int(xmin * w_img); y1 = int(ymin * h_img)
                            x2 = int(xmax * w_img); y2 = int(ymax * h_img)
                            cx = (x1 + x2) // 2
                            
                            if global_frame_depth is not None:
                                raw_dist = get_multi_point_distance(global_frame_depth, x1, y1, x2, y2, h_img, w_img)
                                
                                if raw_dist <= 0.0:
                                    box_h_ratio = (y2 - y1) / h_img
                                    if box_h_ratio > 0.6: raw_dist = 0.8 
                                    elif box_h_ratio > 0.4: raw_dist = 1.5
                                    else: raw_dist = 3.0

                                detected_dist = raw_dist
                                detected_x = cx
                                
                                # AI 예측 및 점수 계산
                                predictor.update(detected_dist, cx)
                                pred_dist, pred_x = predictor.predict_2s_later()
                                risk_score = calculate_risk_score(detected_dist, pred_dist, w_img, cx, pred_x)

                        else:
                            predictor.history.clear()
                            risk_score = 0.0

                        # --- 제어 로직 ---
                        status_text = "IDLE"
                        status_color = (200, 200, 200)

                        if current_goal_msg is not None:
                            # 🛑 위험 점수 높음 (1.5m 이내 접근) -> 정지
                            if risk_score >= RISK_THRESHOLD:
                                if not is_paused_by_guard:
                                    print(f"🛑 [위험] Risk: {risk_score:.1f}% (거리: {detected_dist:.2f}m) -> 정지")
                                    navigator.cancelTask()
                                    is_paused_by_guard = True
                                
                                node.force_stop()
                                status_text = f"STOP (Risk: {int(risk_score)}%)"
                                status_color = (0, 0, 255)
                                
                            else:
                                # ✅ 위험 점수 낮음 -> 재출발 여부 결정
                                if is_paused_by_guard:
                                    # 멈춰있던 상태라면, 점수가 RESUME_THRESHOLD(2.0m 수준) 이하로 떨어져야 출발
                                    if risk_score <= RESUME_THRESHOLD:
                                        print(f"✅ [안전] Risk: {risk_score:.1f}% (거리: {detected_dist:.2f}m) -> 재출발")
                                        current_goal_msg.header.stamp = navigator.get_clock().now().to_msg()
                                        navigator.goToPose(current_goal_msg)
                                        is_paused_by_guard = False
                                    else:
                                        # 아직 2.0m 안쪽임 (1.5m ~ 2.0m 사이) -> 계속 대기
                                        node.force_stop()
                                        status_text = f"WAITING (Move > 2.0m)"
                                        status_color = (0, 165, 255)
                                        continue

                                elif navigator.isTaskComplete():
                                    result = navigator.getResult()
                                    if result == TaskResult.SUCCEEDED or result == TaskResult.CANCELED:
                                        if not is_paused_by_guard:
                                            current_goal_msg = None
                                            print("🎉 [도착] 미션 완료")
                                            status_text = "ARRIVED"
                                            status_color = (0, 255, 0)
                                else:
                                    status_text = "MONITORING"
                                    status_color = (255, 255, 0)

                        # --- GUI ---
                        if SHOW_GUI:
                            gui_counter += 1
                            if gui_counter % GUI_SKIP_RATE == 0:
                                display_frame = global_frame_rgb.copy()
                                
                                bar_width = int(risk_score * 3)
                                bar_color = (0, 255, 0)
                                if risk_score > 50: bar_color = (0, 165, 255)
                                if risk_score >= RISK_THRESHOLD: bar_color = (0, 0, 255)
                                
                                cv2.rectangle(display_frame, (10, 10), (10 + 300, 30), (50, 50, 50), -1)
                                cv2.rectangle(display_frame, (10, 10), (10 + bar_width, 30), bar_color, -1)
                                cv2.putText(display_frame, f"RISK: {int(risk_score)}%", (320, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bar_color, 2)

                                if target_person is not None and detected_x:
                                    # 타겟 박스
                                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), bar_color, 3)
                                    cv2.putText(display_frame, f"{detected_dist:.1f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, bar_color, 2)
                                    
                                    if pred_x:
                                        cv2.arrowedLine(display_frame, (detected_x, h_img//2), (pred_x, h_img//2), (0, 255, 255), 3)

                                cv2.putText(display_frame, f"State: {status_text}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                                cv2.imshow("AI Guard View", display_frame)
                                if cv2.waitKey(1) & 0xFF == ord('q'): break
                            
    except Exception as e: print(f"\n🔥 {e}")
    finally:
        global_running = False
        rclpy.shutdown()
        ros_thread.join()
        if SHOW_GUI: cv2.destroyAllWindows()

if __name__ == '__main__':
    main()