#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from multiprocessing import shared_memory
import struct
import time

GRID_SIZE = 256
T_IN = 10
SHM_NAME = 'bev_sequence_shm'
DATA_SHAPE = (T_IN, GRID_SIZE * GRID_SIZE)
# 데이터 크기: 10 * 256 * 256 * 4 bytes/float32 + 4 bytes(카운터)
DATA_SIZE_BYTES = T_IN * GRID_SIZE * GRID_SIZE * 4 
HEADER_SIZE_BYTES = 4 # 32비트 정수 카운터용
TOTAL_SHM_SIZE = DATA_SIZE_BYTES + HEADER_SIZE_BYTES


class BevClientShm(Node):
    def __init__(self):
        super().__init__("bev_client_shm")
        self.buffer = []

        # Shared Memory 초기화 및 연결
        try:
            # Shared memory block이 없으면 생성하고, 있으면 연결합니다.
            self.shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=TOTAL_SHM_SIZE)
            # 카운터(4바이트) 이후 영역을 NumPy 배열로 매핑
            self.shm_np = np.ndarray(DATA_SHAPE, dtype=np.float32, buffer=self.shm.buf, offset=HEADER_SIZE_BYTES)
            self.counter = self._read_counter()
            self.get_logger().info(f"✅ Shared Memory '{SHM_NAME}' initialized and attached.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Shared Memory. Did you run Docker with --ipc=host? Error: {e}")
            raise

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/bev_frame",
            self.on_bev_frame,
            10
        )
        self.get_logger().info("🧰 BEV Shared Memory Client Node Started.")

    def _read_counter(self):
        """Shared Memory에서 현재 카운터 값을 읽습니다."""
        return struct.unpack('<i', self.shm.buf[0:HEADER_SIZE_BYTES])[0]

    def _write_counter(self, count):
        """Shared Memory에 새로운 카운터 값을 쓰고 flush 합니다."""
        self.shm.buf[0:HEADER_SIZE_BYTES] = struct.pack('<i', count)

    def on_bev_frame(self, msg: Float32MultiArray):
        bev = np.array(msg.data, dtype=np.float32)

        # 롤링 버퍼 업데이트
        self.buffer.append(bev)
        if len(self.buffer) > T_IN:
            self.buffer.pop(0)

        if len(self.buffer) < T_IN:
            return

        seq = np.stack(self.buffer, axis=0) # (10, 256*256)

        try:
            # 1. 데이터를 Shared Memory 영역에 복사
            np.copyto(self.shm_np, seq)
            
            # 2. 카운터를 증가시키고 기록하여 서버에 새로운 데이터가 있음을 알림 (동기화)
            self.counter += 1
            self._write_counter(self.counter)
            
            self.get_logger().info(f"➡️ Data copied to SHM. New counter: {self.counter}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Shared Memory write error: {e}")
        
    def destroy_node(self):
        # 클라이언트 측에서는 연결만 해제합니다.
        if hasattr(self, 'shm'):
             self.shm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BevClientShm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()