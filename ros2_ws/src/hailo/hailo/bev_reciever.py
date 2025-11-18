#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import struct
import time
from multiprocessing import shared_memory

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


GRID = 256
T_OUT = 10
SHM_NAME = 'bev_output_shm'
HEADER_BYTES = 4


# ============================================================
#   Shared Memory Reader  (Auto Wait for SHM)
# ============================================================

class BevOutputShmReader:
    def __init__(self):
        self.shm = self.wait_for_shm()

        print(f"[SHM] Connected to '{SHM_NAME}'")

        size_flat = GRID * GRID * T_OUT

        self.flat_view = np.ndarray(
            (size_flat,),
            dtype=np.float32,
            buffer=self.shm.buf,
            offset=HEADER_BYTES
        )

        self.last_counter = -1

    # -------------------------------------------
    #   🔥 SHM 기다리는 함수 (추가됨)
    # -------------------------------------------
    def wait_for_shm(self):
        print(f"[SHM] Waiting for '{SHM_NAME}' to be created...")

        while True:
            try:
                shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
                return shm
            except FileNotFoundError:
                time.sleep(0.2)  # 200ms retry

    def read_counter(self):
        return struct.unpack('<i', self.shm.buf[0:4])[0]

    def read_latest(self):
        counter = self.read_counter()

        if counter == self.last_counter:
            return None, counter

        arr = self.flat_view.copy()
        arr = arr.reshape((GRID, GRID, T_OUT))

        self.last_counter = counter
        return arr, counter

    def close(self):
        self.shm.close()


# ============================================================
#      ROS2 Node → Publish OccupancyGrid
# ============================================================

class BevFuturePublisher(Node):
    def __init__(self):
        super().__init__("bev_future_publisher")
        self.get_logger().info("BEV Future OccupancyGrid Publisher started.")

        self.reader = BevOutputShmReader()

        self.pub = self.create_publisher(OccupancyGrid, "/bev_future_grid", 10)

        # 20Hz polling (50ms)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        data, counter = self.reader.read_latest()

        if data is None:
            return

        future_grid = data[:, :, -1]  # 256×256

        msg = self.numpy_to_occupancy_grid(future_grid, counter)
        self.pub.publish(msg)

        self.get_logger().info(f"[{counter}] Published BEV OccupancyGrid")

    def numpy_to_occupancy_grid(self, grid: np.ndarray, counter: int):
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.2

        msg.info.width = GRID
        msg.info.height = GRID

        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0

        g = grid.copy()
        g = np.clip(g, -5, 5)
        g = (g - g.min()) / (g.max() - g.min() + 1e-6)
        g = (g * 100).astype(np.int8)

        msg.data = g.flatten().tolist()

        return msg

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()


# ============================================================
#   Main
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = BevFuturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
