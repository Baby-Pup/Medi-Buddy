import numpy as np
import os
import struct
import time
from functools import partial
from multiprocessing import shared_memory

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
plt.ion()

from HailoInfer import HailoInfer


# ==============================
#  CONFIG
# ==============================

GRID = 256
T_IN = 10
T_OUT = 10

INPUT_SHM_NAME = 'bev_sequence_shm'
OUTPUT_SHM_NAME = 'bev_output_shm'
HEADER_BYTES = 4


# ======================================================
#  1. Shared Memory Reader (BEV Sequence 10-frame input)
# ======================================================

class SharedMemoryReader:
    def __init__(self):
        self.current_counter = -1

        # -------------------------------
        #   🔥 SHM 생성될 때까지 대기
        # -------------------------------
        self.shm = self.wait_for_shm()

        self.flat_view = np.ndarray(
            (T_IN * GRID * GRID,),
            dtype=np.float32,
            buffer=self.shm.buf,
            offset=HEADER_BYTES
        )

        print(f"[SHM Reader] Connected to '{INPUT_SHM_NAME}'")

    # --------------------------
    #   SHM 기다리는 함수
    # --------------------------
    def wait_for_shm(self):
        print(f"[SHM Reader] Waiting for '{INPUT_SHM_NAME}' to be created...")

        while True:
            try:
                shm = shared_memory.SharedMemory(
                    name=INPUT_SHM_NAME,
                    create=False
                )
                return shm
            except FileNotFoundError:
                time.sleep(0.2)   # 200ms retry

    def _read_counter(self):
        return struct.unpack('<i', self.shm.buf[0:4])[0]

    def read_latest(self):
        new_counter = self._read_counter()

        if new_counter > self.current_counter:
            data = self.flat_view.copy()
            data = data.reshape((T_IN, GRID, GRID))
            self.current_counter = new_counter
            return data, new_counter

        return None, new_counter

    def close(self):
        self.shm.close()


# ======================================================
#  2. Shared Memory Writer (Model Output 10 frames)
# ======================================================

class BevOutputShmWriter:
    def __init__(self):
        size = HEADER_BYTES + GRID * GRID * T_OUT * 4  # float32

        # 기존 shm 있으면 삭제
        try:
            old = shared_memory.SharedMemory(name=OUTPUT_SHM_NAME)
            old.close()
            old.unlink()
        except:
            pass

        # 새 shm 만들기
        self.shm = shared_memory.SharedMemory(
            name=OUTPUT_SHM_NAME,
            create=True,
            size=size
        )

        self.counter = 0

        self.flat_view = np.ndarray(
            (GRID * GRID * T_OUT,),
            dtype=np.float32,
            buffer=self.shm.buf,
            offset=HEADER_BYTES
        )

        print(f"[SHM Writer] Created '{OUTPUT_SHM_NAME}', size={size} bytes")

    def write(self, arr_3d):
        """ arr_3d shape = (256,256,10) """
        self.counter += 1

        # counter 기록
        self.shm.buf[0:4] = struct.pack('<i', self.counter)

        # 데이터 저장
        self.flat_view[:] = arr_3d.flatten()

    def close(self):
        self.shm.close()
        self.shm.unlink()


# ======================================================
#  3. Inference Callback Handler
# ======================================================

def inference_result_handler(
        *, bindings_list, completion_info, read_counter,
        start_time, output_writer):

    latency = (time.time() - start_time) * 1000

    # 모델 출력 shape = (256,256,10)
    output = bindings_list[0].output().get_buffer()

    # → SHM output writer에 저장
    output_writer.write(output)

    # 마지막 프레임을 시각화
    last_frame = output[:, :, -1]

    # plt.clf() # 속도 저하 가능성
    # plt.imshow(last_frame, cmap='hot', interpolation='nearest') # 속도 저하 가능성
    # plt.title(f"Future Occupancy (t=9) | frame={read_counter}") # 속도 저하 가능성
    # plt.pause(0.001) # 속도 저하 가능성

    print(f"[{read_counter}] Latency={latency:.2f} ms")


# ======================================================
#  4. Main Loop
# ======================================================

def main_infer(hef_path, batch_size):
    # Hailo 모델 준비
    hailo = HailoInfer(
        hef_path=hef_path,
        batch_size=batch_size,
        input_type="FLOAT32",
        output_type="FLOAT32"
    )
    print("[Hailo] Inference Engine Ready.")

    # Shared Memory
    reader = SharedMemoryReader()
    writer = BevOutputShmWriter()

    print("🚀 Running inference loop...")

    try:
        while True:
            time.sleep(0.001)      # Polling 1ms
            bev_seq, counter = reader.read_latest()

            if bev_seq is None:
                continue

            start_time = time.time()

            hailo.run(
                input_batch=[bev_seq],
                inference_callback_fn=partial(
                    inference_result_handler,
                    read_counter=counter,
                    start_time=start_time,
                    output_writer=writer
                )
            )

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        hailo.close()
        reader.close()
        writer.close()


# ======================================================
#  Entry Point
# ======================================================

if __name__ == "__main__":
    main_infer(
        hef_path="/home/dog/hailo-rpi5-examples/bev_future_tsm_unet_online.hef",
        batch_size=1
    )
