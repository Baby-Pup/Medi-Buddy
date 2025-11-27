#!/usr/bin/env python3
# navigation/semantic_router_node.py

import os
import yaml
from typing import Dict, Tuple
from collections import deque
from math import cos, sin

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from interfaces.msg import DestinationRequest, NavEvent

from ament_index_python.packages import get_package_share_directory


def load_yaml(path: str) -> Dict[str, Tuple[float, float, float]]:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return {str(k).lower(): v for k, v in data.items()}


class SemanticRouterNode(Node):
    def __init__(self):
        super().__init__("semantic_router")

        # 🚀 /nav_status 제거: 이제 _nav_enabled는 항상 True
        self._nav_enabled = True # 항상 주행 가능 상태로 간주

        # ========== POI 파일 로드 ==========
        try:
            default_poi_path = os.path.join(
                get_package_share_directory("navigation"), "config", "poi_map.yaml"
            )
        except Exception:   
            default_poi_path = os.path.expanduser(
                "~/ros2_ws/src/navigation/config/poi_map.yaml"
            )

        self.declare_parameter("poi_yaml_path", default_poi_path)
        self.poi_yaml_path = self.get_parameter("poi_yaml_path").get_parameter_value().string_value
        os.makedirs(os.path.dirname(self.poi_yaml_path), exist_ok=True)
        self._poi, self._poi_mtime = {}, 0.0

        # ========== 홈 포인트 로드 ==========
        nav2_param_path = os.path.join(
            get_package_share_directory("navigation"),
            "config",
            "nav2_params.yaml"
        )
        self._home_pose = self._load_home_pose(nav2_param_path)
        self.get_logger().info(f"🏠 홈 포인트 로드 완료: {self._home_pose}")

        # ========== 내부 상태 ==========
        self._route_list = deque()        # 남은 고정 루트 (다음 목적지는 맨 앞)
        self._pending_goal = None         # Detour 후 복귀 대상(규칙에 따라 b 또는 c)
        self._current_goal = None         # 현재 Nav2에서 주행 중인 목적지 이름
        self._detour_mode = False         # Detour 상태 여부
        self._current_goal_handle = None  # Nav2 goal handle
        self._arrived_waiting = False     # 마지막 목적지 도착 후 다음 출발 대기 상태

        # ========== ROS 인터페이스 ==========
        self._dest_sub = self.create_subscription(
            DestinationRequest, "/destination_request", self.on_destination_request, 10
        )
        self._route_sub = self.create_subscription(
            String, "/destination_list", self.on_destination_list, 10
        )
        # face_detection_status 
        self._trigger_sub = self.create_subscription(
            Bool, "/face_detection_status", self.on_face_encoded, 10
        )
        # ❌ /nav_status 구독 삭제

        self._event_pub = self.create_publisher(NavEvent, "/nav_event", 10)
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.get_logger().info("✅ semantic_router_node ready (nav_status check removed)")

    # ------------------------------------------------
    # Utility
    # ------------------------------------------------
    def _publish_event(self, event_type: str, target: str):
        msg = NavEvent()
        msg.event_type = event_type
        msg.target_name = target
        self._event_pub.publish(msg)
        self.get_logger().info(f"[EVENT] {event_type} -> {target}")

    def _ensure_poi_loaded(self):
        try:
            mtime = os.path.getmtime(self.poi_yaml_path)
        except FileNotFoundError:
            self.get_logger().warn(f"POI 파일이 없습니다: {self.poi_yaml_path}")
            return
        if mtime != self._poi_mtime:
            try:
                self._poi = load_yaml(self.poi_yaml_path)
                self._poi_mtime = mtime
                self.get_logger().info(f"POI {len(self._poi)}개 로드/갱신")
            except Exception as e:
                self.get_logger().error(f"POI 로드 실패: {e}")

    def _get_pose_for_name(self, name: str):
        """POI 이름을 좌표로 변환"""
        self._ensure_poi_loaded()
        key = name.lower()
        if key not in self._poi:
            self.get_logger().warn(f"POI '{name}' 미등록 (poi_map.yaml 확인)")
            return None
        x, y, z = self._poi[key]
        return (float(x), float(y), float(z))

    def _load_home_pose(self, yaml_path: str):
        """nav2_params.yaml에서 amcl.initial_pose 값을 읽어 홈 포인트로 설정"""
        try:
            with open(yaml_path, "r") as f:
                params = yaml.safe_load(f)
            amcl_params = params.get("amcl", {}).get("ros__parameters", {})
            initial_pose = amcl_params.get("initial_pose", {})

            x = float(initial_pose.get("x", 0.0))
            y = float(initial_pose.get("y", 0.0))
            yaw = float(initial_pose.get("yaw", 0.0))

            self.get_logger().info(f"✅ nav2_params.yaml에서 홈 로드: x={x}, y={y}, yaw={yaw}")
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"⚠️ 홈 좌표 로드 실패 (기본값 0,0,0 사용): {e}")
            return (0.0, 0.0, 0.0)

    # ------------------------------------------------
    # Callbacks
    # ------------------------------------------------
    def on_destination_list(self, msg: String):
        items = [x.strip().lower() for x in msg.data.split(",") if x.strip()]
        if not items:
            self.get_logger().warn("빈 destination_list 수신—무시")
            return

        self._route_list = deque(items)
        self._detour_mode = False
        self._pending_goal = None
        self._arrived_waiting = False

        self.get_logger().info(f"📜 루트 수신: {list(self._route_list)}")

        # 🚀 nav_status 체크 제거 후 바로 출발 시도
        self._go_next_in_route()

    def on_destination_request(self, req: DestinationRequest):
        """모든 단일 목적지 요청 (루트 내/외 구분)"""
        name = (req.destination_name or "").strip().lower()
        if not name:
            return

        # 🚀 nav_status 체크 제거
        # if not self._nav_enabled:
        #     self.get_logger().warn(f"🚫 nav_status=False → Detour 요청 '{name}' 보류")
        #     return

        # 루트 내 목적지면 루트 로직으로 처리되니 여기선 무시
        if name in self._route_list:
            self.get_logger().info(f"루트 내 목적지 요청 '{name}' 무시 (이미 관리 중)")
            return

        # ===== Detour 분기: '주행 중' vs '도착 후 대기' =====
        if self._arrived_waiting:
            self._current_goal = None  # 🔧 중요: 마지막 목적지 도착 후 detour 시 현재 goal 리셋
            # ❗ 도착 후(예: b 도착) 화장실 → 복귀 시 '다음 목적지(c)'로 가야 함
            if self._route_list:
                self._pending_goal = self._route_list[0]  # 다음 목적지 미리 저장
                self.get_logger().info(f"🚧 Detour(대기상태): 복귀 대상=다음 루트 '{self._pending_goal}'")
            else:
                self._pending_goal = "__home__"
                self.get_logger().info("🚧 Detour(대기상태): 남은 루트 없음 → 홈으로 복귀 예정")
        else:
            # ❗ 주행 중(a→b) 화장실 → 복귀 시 '현재 가던 목적지(b)'로 가야 함
            if self._current_goal:
                self._pending_goal = self._current_goal
                self.get_logger().info(f"🚧 Detour(주행중): 복귀 대상=현재 목표 '{self._pending_goal}'")
            else:
                # 이론상 거의 없음(주행중인데 current_goal이 비었다면)
                self._pending_goal = self._route_list[0] if self._route_list else None
                self.get_logger().info(f"🚧 Detour(주행중): 복귀 대상 자동보정='{self._pending_goal}'")

        self._handle_detour(name)

    def on_face_encoded(self, msg):
        """/face_encoded 콜백: True가 들어오면 주행 허가 및 다음 이동 트리거"""
        # 🚀 _nav_enabled가 항상 True이므로, msg.data: True이면 이동 로직 수행
        if msg.data:
            self.get_logger().info("✅ /face_encoded=True 수신 — 주행 트리거됨")
            
            # Detour 복귀 로직
            if self._detour_mode and self._pending_goal:
                self.get_logger().info(f"✅ Detour 완료 — 복귀: {self._pending_goal}")
                goal = self._pending_goal
                self._pending_goal = None
                self._detour_mode = False

                if goal == "__home__":
                    self._go_home()
                else:
                    self._go_to(goal)
                return

            # 일반 루트 진행
            self._go_next_in_route()
        else:
            # False가 들어오면 무시하고 다음 True를 기다림 (주행 상태는 유지)
            self.get_logger().info("🔕 /face_encoded=False 수신 — 무시됨")


    # ------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------
    def _go_next_in_route(self):
        # ❌ nav_enabled 체크 제거
        # if not self._nav_enabled:
        #     self.get_logger().warn("🚫 nav_status=False → 루트 이동 보류")
        #     self._arrived_waiting = True
        #     return

        if not self._route_list:
            self.get_logger().info("🎯 모든 루트 완료! 홈으로 복귀")
            self._go_home()
            return

        next_dest = self._route_list.popleft()
        self._go_to(next_dest)

    def _handle_detour(self, name: str):
        """임시 목적지(Detour) 처리"""
        # ❌ nav_enabled 체크 제거
        # if not self._nav_enabled:
        #     self.get_logger().warn(f"🚫 nav_status=False → Detour '{name}' 보류")
        #     return
            
        # 현재 Nav2 goal 취소 (주행 중이라면)
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        # Detour 모드 진입
        self._detour_mode = True
        self._go_to(name)

    def _go_to(self, name: str):
        """Nav2 액션 클라이언트로 이동 명령"""
        # ❌ nav_enabled 체크 제거
        # if not self._nav_enabled:
        #     self.get_logger().warn(f"🚫 nav_status=False → 이동 '{name}' 보류")
        #     self._arrived_waiting = True
        #     return

        pose = self._get_pose_for_name(name)
        if not pose:
            self._publish_event("CANCELLED", name)
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose 액션 서버 미가용")
            self._publish_event("CANCELLED", name)
            return

        x, y, z = pose
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        goal = NavigateToPose.Goal()
        goal.pose = ps

        # 이동 시작: 도착 대기 상태 해제
        self._arrived_waiting = False
        self._current_goal = name
        self._publish_event("START", name)

        send_future = self._nav_client.send_goal_async(goal, feedback_callback=lambda fb: None)
        send_future.add_done_callback(lambda fut: self._on_goal_response(name, fut))

    def _go_home(self):
        """모든 목적지 완료 후 홈 포인트로 복귀"""
        # ❌ nav_enabled 체크 제거
        # if not self._nav_enabled:
        #     self.get_logger().warn("🚫 nav_status=False → 홈 복귀 보류")
        #     self._arrived_waiting = True
        #     return

        x, y, yaw = self._home_pose
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=sin(yaw / 2.0), w=cos(yaw / 2.0)
        )

        goal = NavigateToPose.Goal()
        goal.pose = ps

        self._publish_event("RETURN_HOME", "home")

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose 서버 미가용 — 홈 복귀 실패")
            return

        # 이동 시작: 도착 대기 상태 해제
        self._arrived_waiting = False
        self._current_goal = "home"

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda fut: self.get_logger().info("🏁 홈 복귀 명령 완료"))

    def _on_goal_response(self, name, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal 전송 실패: {e}")
            self._publish_event("CANCELLED", name)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal 거부됨: {name}")
            self._publish_event("CANCELLED", name)
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info(f"Goal 수락됨 — '{name}'로 이동 중")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(name, fut))

    def _on_result(self, name, future):
        self._current_goal_handle = None
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"결과 수신 실패: {e}")
            self._publish_event("CANCELLED", name)
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"목표 '{name}' 도착")
            self._publish_event("ARRIVED", name)

            # 도착 → 다음 출발 전 대기 상태로 전환
            self._current_goal = None
            self._arrived_waiting = True

        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self.get_logger().warn(f"주행 실패/취소: {name}")
            self._publish_event("CANCELLED", name)
            # 실패 시에도 대기 상태로 두고, 외부 트리거에 맡김
            self._current_goal = None
            self._arrived_waiting = True
        else:
            self.get_logger().warn(f"알 수 없는 상태 코드: {status}")
            self._publish_event("CANCELLED", name)
            self._current_goal = None
            self._arrived_waiting = True

    def destroy_node(self):
        self.get_logger().info("🧹 semantic_router 종료")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()