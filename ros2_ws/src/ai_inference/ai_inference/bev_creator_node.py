#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# =========================
# CONFIG
# =========================
GRID_SIZE = 128
RESOLUTION = 0.1   # 0.1m per cell → half-size = 6.4m
CENTER = GRID_SIZE // 2     # 64
RANGE_MAX = 6.4             # 128 * 0.1 / 2  (전방 6.4m)
THETA_MIN = -np.pi/2        # -90 deg
THETA_MAX = +np.pi/2        # +90 deg


# =====================================================
# 180° CROPPING (학습과 동일)
# =====================================================
def crop_front_180(ranges, angle_min, angle_inc):
    angles = angle_min + np.arange(len(ranges)) * angle_inc
    mask = (angles >= THETA_MIN) & (angles <= THETA_MAX)
    return ranges[mask], angles[mask]


# =====================================================
# BEV 변환 (128×128 / front-only / 학습과 동일)
# =====================================================
def lidar_to_bev_front(ranges, angles):
    bev = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    # 1. 유효 범위 필터링
    valid = (ranges > 0.03) & (ranges < RANGE_MAX) & np.isfinite(ranges)
    if not np.any(valid):
        return bev

    r = ranges[valid]
    th = angles[valid]

    # 2. Polar → Cartesian
    x = r * np.cos(th)   # front (+x)
    y = r * np.sin(th)   # left (+y)

    # 3. Cartesian → grid index
    rows = np.floor(CENTER - (x / RESOLUTION)).astype(int)
    cols = np.floor(CENTER - (y / RESOLUTION)).astype(int)

    mask = (rows >= 0) & (rows < GRID_SIZE) & (cols >= 0) & (cols < GRID_SIZE)
    bev[rows[mask], cols[mask]] = 1.0

    return bev


# =====================================================
# NODE
# =====================================================
class BevCreator(Node):
    def __init__(self):
        super().__init__("bev_creator")

        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.on_scan,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/bev_frame",
            10
        )

        self.get_logger().info("📡 BEV Creator (128x128, Front 180°) Started.")

    def on_scan(self, msg: LaserScan):

        # ---------- 동일한 180° 크롭 적용 ----------
        ranges = np.array(msg.ranges, dtype=np.float32)
        cropped_ranges, cropped_angles = crop_front_180(
            ranges,
            msg.angle_min,
            msg.angle_increment
        )

        # angle_increment는 LiDAR 기본값 유지
        bev = lidar_to_bev_front(cropped_ranges, cropped_angles)

        # ---------- Publish ----------
        out = Float32MultiArray()
        out.data = bev.flatten().tolist()
        self.pub.publish(out)


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = BevCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
