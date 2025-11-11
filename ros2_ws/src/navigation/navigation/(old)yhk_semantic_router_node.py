#!/usr/bin/env python3
# navigation/semantic_router_node.py

import os
import yaml
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose

from interfaces.msg import DestinationRequest, NavEvent
from ament_index_python.packages import get_package_share_directory


def load_yaml(path: str) -> Dict[str, Tuple[float, float, float]]:
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    # 키를 소문자로 표준화
    norm = {str(k).lower(): v for k, v in data.items()}
    return norm


class SemanticRouterNode(Node):
    def __init__(self):
        super().__init__('semantic_router')

        # 파라미터: POI YAML 경로 (기본: navigation 패키지의 share/config/poi_map.yaml)
        try:
            default_poi_path = os.path.join(
                get_package_share_directory('navigation'), 'config', 'poi_map.yaml'
            )
        except Exception:
            # 개발 단계에서 src 경로를 바로 쓰고 싶은 경우를 대비
            default_poi_path = os.path.expanduser('~/ros2_ws/src/navigation/config/poi_map.yaml')

        self.declare_parameter('poi_yaml_path', default_poi_path)
        self.poi_yaml_path = self.get_parameter('poi_yaml_path').get_parameter_value().string_value
        os.makedirs(os.path.dirname(self.poi_yaml_path), exist_ok=True)

        self.get_logger().info(f'📍 POI 파일: {self.poi_yaml_path}')

        # 인터페이스
        self._dest_sub = self.create_subscription(
            DestinationRequest, '/destination_request', self.on_destination, 10
        )
        self._event_pub = self.create_publisher(NavEvent, '/nav_event', 10)

        # Nav2 액션 클라이언트
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_handle = None

        # 캐시
        self._poi = {}
        self._poi_mtime = 0.0

        self.get_logger().info('✅ semantic_router_node ready (listening on /destination_request)')

    # ---------- 유틸 ----------
    def _publish_event(self, event_type: str, target: str):
        msg = NavEvent()
        msg.event_type = event_type
        msg.target_name = target
        self._event_pub.publish(msg)
        self.get_logger().info(f'[EVENT] {event_type} -> {target}')

    def _ensure_poi_loaded(self):
        try:
            mtime = os.path.getmtime(self.poi_yaml_path)
        except FileNotFoundError:
            if not self._poi:
                self.get_logger().warn(f'POI 파일이 없습니다: {self.poi_yaml_path}')
            return
        if mtime != self._poi_mtime:
            try:
                self._poi = load_yaml(self.poi_yaml_path)
                self._poi_mtime = mtime
                self.get_logger().info(f'POI {len(self._poi)}개 로드/갱신')
            except Exception as e:
                self.get_logger().error(f'POI 로드 실패: {e}')

    # ---------- 콜백 ----------
    def on_destination(self, req: DestinationRequest):
        name = (req.destination_name or '').strip().lower()
        if not name:
            self.get_logger().warn('빈 destination_name 수신—무시')
            return

        # POI 로드/갱신
        self._ensure_poi_loaded()
        if name not in self._poi:
            self.get_logger().warn(f'POI 미등록: "{name}" (poi_map.yaml 확인)')
            self._publish_event('CANCELLED', name)
            return

        x, y, z = self._poi[name]
        self.get_logger().info(f'목표 "{name}" → ({x:.3f}, {y:.3f}, {z:.3f})')

        # Nav2 액션 서버 대기
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 액션 서버 미가용')
            self._publish_event('CANCELLED', name)
            return

        # 목표 포즈 구성 (map frame 기준)
        ps = PoseStamped()
        ps.header = Header()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # yaw=0

        goal = NavigateToPose.Goal()
        goal.pose = ps

        # 기존 Goal 있으면 취소 시도
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        self._publish_event('START', name)

        # 비동기 전송
        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_feedback(name, fb)
        )
        send_future.add_done_callback(lambda fut: self._on_goal_response(name, fut))

    def _on_feedback(self, name: str, feedback_msg):
        # 필요시 진행률/거리 출력 (quiet 모드 유지)
        pass

    def _on_goal_response(self, name: str, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal 전송 실패: {e}')
            self._publish_event('CANCELLED', name)
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal 거부됨')
            self._publish_event('CANCELLED', name)
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal 수락됨—주행 시작')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(name, fut))

    def _on_result(self, name: str, future):
        self._current_goal_handle = None
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'결과 수신 실패: {e}')
            self._publish_event('CANCELLED', name)
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 도착')
            self._publish_event('ARRIVED', name)
        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self.get_logger().warn(f'주행 실패/취소 (status={status})')
            self._publish_event('CANCELLED', name)
        else:
            self.get_logger().warn(f'알 수 없는 상태 코드: {status}')
            self._publish_event('CANCELLED', name)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
