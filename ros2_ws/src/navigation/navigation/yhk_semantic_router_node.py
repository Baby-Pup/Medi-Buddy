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
    """ YAML 키를 모두 strip().lower() 로 통일 """
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return {str(k).strip().lower(): v for k, v in data.items()}


class SemanticRouterNode(Node):
    def __init__(self):
        super().__init__("semantic_router")

        self._nav_enabled = True

        # POI 파일 로드
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

        # home pose 로드는 유지하지만 자동 이동에는 사용하지 않음
        nav2_param_path = os.path.join(
            get_package_share_directory("navigation"),
            "config",
            "nav2_params.yaml"
        )
        self._home_pose = self._load_home_pose(nav2_param_path)

        # 내부 변수
        self._route_list = deque()
        self._pending_goal = None
        self._current_goal = None
        self._detour_mode = False
        self._current_goal_handle = None
        self._arrived_waiting = False

        # ROS interfaces
        self._dest_sub = self.create_subscription(
            String, "/detour", self.on_destination_request, 10
        )
        self._route_sub = self.create_subscription(
            String, "/destination_list", self.on_destination_list, 10
        )
        self._trigger_sub = self.create_subscription(
            Bool, "/face_detection_status", self.on_face_encoded, 10
        )

        self._event_pub = self.create_publisher(NavEvent, "/nav_event", 10)
        self._current_dest_pub = self.create_publisher(String, "/current_destination", 10)
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    # -------------------------------------------------------
    # Common: name normalize
    # -------------------------------------------------------
    def _normalize(self, name: str) -> str:
        """모든 목적지 이름을 strip().lower()로 정규화"""
        if not name:
            return ""
        return name.strip().lower()

    # -------------------------------------------------------
    # YAML POI Loader
    # -------------------------------------------------------
    def _ensure_poi_loaded(self):
        try:
            mtime = os.path.getmtime(self.poi_yaml_path)
        except FileNotFoundError:
            self.get_logger().warn(f"POI 없음: {self.poi_yaml_path}")
            return

        if mtime != self._poi_mtime:
            self._poi = load_yaml(self.poi_yaml_path)
            self._poi_mtime = mtime
            self.get_logger().info(f"POI {len(self._poi)}개 로드/갱신")

    def _get_pose_for_name(self, name: str):
        name = self._normalize(name)
        self._ensure_poi_loaded()
        if name not in self._poi:
            self.get_logger().warn(f"POI '{name}' 미등록")
            return None
        x, y, z = self._poi[name]
        return float(x), float(y), float(z)

    def _load_home_pose(self, yaml_path: str):
        try:
            with open(yaml_path, "r") as f:
                params = yaml.safe_load(f)
            amcl_params = params.get("amcl", {}).get("ros__parameters", {})
            initial_pose = amcl_params.get("initial_pose", {})

            x = float(initial_pose.get("x", 0.0))
            y = float(initial_pose.get("y", 0.0))
            yaw = float(initial_pose.get("yaw", 0.0))

            self.get_logger().info(f"홈 포인트 로드됨 (x={x}, y={y}, yaw={yaw})")
            return (x, y, yaw)
        except Exception:
            return (0.0, 0.0, 0.0)

    # -------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------
    def on_destination_list(self, msg: String):
        items = [self._normalize(x) for x in msg.data.split(",") if x.strip()]
        if not items:
            self.get_logger().warn("빈 destination_list 무시됨")
            return

        self._route_list = deque(items)
        self._detour_mode = False
        self._pending_goal = None
        self._arrived_waiting = False

        self.get_logger().info(f"📜 루트 수신: {list(self._route_list)}")

        self._go_next_in_route()

    def on_destination_request(self, msg: String):
        name = self._normalize(msg.data)
        if not name:
            return

        if name in self._route_list:
            return

        # detour 복귀 목표 저장
        if self._arrived_waiting:
            self._pending_goal = self._route_list[0] if self._route_list else None
        else:
            self._pending_goal = self._current_goal or (self._route_list[0] if self._route_list else None)

        self._handle_detour(name)

    def on_face_encoded(self, msg):
        if msg.data:
            if self._detour_mode and self._pending_goal:
                goal = self._pending_goal
                self._pending_goal = None
                self._detour_mode = False

                if goal != "__home__":      # home 이동 금지
                    self._go_to(goal)
                return

            # route 있을 때만 다음 목적지로 이동
            if self._route_list:
                self._go_next_in_route()

    # -------------------------------------------------------
    # Navigation
    # -------------------------------------------------------
    def _go_next_in_route(self):
        if not self._route_list:
            return  # home 이동 삭제

        next_dest = self._route_list.popleft()
        self._publish_current_destination(next_dest) 
        self._go_to(next_dest)

    def _handle_detour(self, name: str):
        name = self._normalize(name)

        if self._current_goal_handle:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        self._detour_mode = True
        self._publish_current_destination(name)
        self._go_to(name)

    def _go_to(self, name: str):
        name = self._normalize(name)

        pose = self._get_pose_for_name(name)
        if not pose:
            self._publish_event("CANCELLED", name)
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
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

        self._arrived_waiting = False
        self._current_goal = name
        self._publish_event("START", name)

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(name, fut))

    # -------------------------------------------------------
    # Home (보존하되 자동 호출 없음)
    # -------------------------------------------------------
    def _go_home(self):
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

        self._arrived_waiting = False
        self._current_goal = "home"

        self._nav_client.send_goal_async(goal)

    # -------------------------------------------------------
    # Action Callbacks
    # -------------------------------------------------------
    def _on_goal_response(self, name, future):
        try:
            gh = future.result()
        except Exception:
            self._publish_event("CANCELLED", name)
            return

        if not gh.accepted:
            self._publish_event("CANCELLED", name)
            return

        self._current_goal_handle = gh
        gh.get_result_async().add_done_callback(lambda fut: self._on_result(name, fut))

    def _on_result(self, name, future):
        self._current_goal_handle = None
        try:
            result = future.result()
        except Exception:
            self._publish_event("CANCELLED", name)
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_event("ARRIVED", name)
        else:
            self._publish_event("CANCELLED", name)

        self._current_goal = None
        self._arrived_waiting = True

    # -------------------------------------------------------
    def _publish_event(self, event_type: str, target: str):
        msg = NavEvent()
        msg.event_type = event_type
        msg.target_name = target
        self._event_pub.publish(msg)
    
    def _publish_current_destination(self, name: str):
        msg = String()
        msg.data = name
        self._current_dest_pub.publish(msg)
        self.get_logger().info(f"[PUB] current_destination → {name}")



def main(args=None):
    rclpy.init(args=args)
    node = SemanticRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
