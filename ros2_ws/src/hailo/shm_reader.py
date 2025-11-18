# shm_reader.py (Run inside Docker Container)

import multiprocessing.shared_memory as shm
import numpy as np
import time
import sys
import os

# --- Data Shared Memory Setup ---
SHM_DATA_NAME = "hailo_test_shm"
DATA_SHAPE = (4, 4)
DATA_DTYPE = np.int64
DATA_SIZE = DATA_SHAPE[0] * DATA_SHAPE[1] * DATA_DTYPE().itemsize

# --- State Shared Memory Setup (Synchronization Counter) ---
SHM_STATE_NAME = "hailo_test_state"

# Minimum polling delay (e.g., 1ms)
POLL_DELAY = 0.001 

print(f"--- Container: Low-Latency Shared Memory Reader Running ---")

shm_data = None
shm_state = None

try:
    # 1. Open State Shared Memory (Counter)
    print(f"⏳ Waiting for shared memory '{SHM_STATE_NAME}' from host...")
    while True:
        try:
            shm_state = shm.SharedMemory(name=SHM_STATE_NAME, create=False)
            frame_counter = np.ndarray((1,), dtype=DATA_DTYPE, buffer=shm_state.buf)
            break
        except FileNotFoundError:
            # Check if host has terminated the segment
            if not os.path.exists(f"/dev/shm/{SHM_STATE_NAME}"):
                raise FileNotFoundError(f"Shared memory segment '{SHM_STATE_NAME}' not found. Host might have exited.")
            time.sleep(0.1)

    # 2. Open Data Shared Memory
    shm_data = shm.SharedMemory(name=SHM_DATA_NAME, create=False)
    shared_array = np.ndarray(DATA_SHAPE, dtype=DATA_DTYPE, buffer=shm_data.buf)
    print(f"✅ Shared Memory segment connected successfully. Starting data read.")

    last_index = -1
    frames_read = 0
    
    # 3. Data Reading Loop (Counter-based synchronization)
    while True:
        current_index = frame_counter[0]
        
        # Read data ONLY when the counter has incremented
        if current_index > last_index:
            # Read Data (Receiving Hailo Results)
            current_value = shared_array[0, 0] 
            
            # Update read count
            frames_read += 1
            last_index = current_index
            
            sys.stdout.write(f"[{frames_read:05d}] Container read value: {current_value} (Index: {current_index})\r")
            sys.stdout.flush()
        
        # Wait briefly until the counter changes
        time.sleep(POLL_DELAY) 

except KeyboardInterrupt:
    print("\n--- Container Program Interrupted ---")
except Exception as e:
    # Catch error if the host destroys the segment while reading
    if "No such file or directory" in str(e):
        print("\nHost terminated the shared memory segment. Reader exiting.")
    else:
        print(f"\nContainer reading error: {e}")

finally:
    # 4. Release Resources
    print("\n--- Container: Releasing resources ---")
    if shm_data:
        shm_data.close()
    if shm_state:
        shm_state.close()
    print("✅ Shared Memory view released successfully.")
    sys.exit(0)
