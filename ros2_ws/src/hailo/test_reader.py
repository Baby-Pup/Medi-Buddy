# test_reader.py
# Run this script inside the Docker Container ('docker exec -it Intelpi /bin/bash' 후 실행).

import multiprocessing.shared_memory as shm
import numpy as np
import time
import sys
import os

# --- Shared Memory Configuration (MUST match hailo_writer.py) ---
MAX_DETECTIONS = 40
DATA_DTYPE = np.float64
DATA_SHAPE = (MAX_DETECTIONS, 6) # 10 rows, 6 columns
DATA_SIZE = DATA_SHAPE[0] * DATA_SHAPE[1] * DATA_DTYPE().itemsize

SHM_DATA_NAME = "hailo_data_shm"
SHM_STATE_NAME = "hailo_state_shm"

# Minimum polling delay (e.g., 1ms)
POLL_DELAY = 0.001 

print(f"--- Container: Low-Latency SHM Reader Running ---")
print(f"Expected Data Shape: {DATA_SHAPE}, Total Size: {DATA_SIZE} bytes")

shm_data = None
shm_state = None

try:
    # 1. Open State Shared Memory (Counter)
    print(f"⏳ Waiting for SHM segments from host...")
    while True:
        try:
            shm_state = shm.SharedMemory(name=SHM_STATE_NAME, create=False)
            frame_counter = np.ndarray((1,), dtype=np.int64, buffer=shm_state.buf)
            break
        except FileNotFoundError:
            # Check for host exit condition
            if not os.path.exists(f"/dev/shm/{SHM_STATE_NAME}"):
                raise FileNotFoundError(f"State segment not found. Host might have exited.")
            time.sleep(0.1)

    # 2. Open Data Shared Memory
    shm_data = shm.SharedMemory(name=SHM_DATA_NAME, create=False)
    # Create the view using the exact same shape and dtype
    shared_array_view = np.ndarray(DATA_SHAPE, dtype=DATA_DTYPE, buffer=shm_data.buf)
    
    print(f"✅ SHM Segments connected successfully. Starting data read.")

    last_index = -1
    frames_read = 0
    
    # 3. Data Reading Loop (Counter-based synchronization)
    while True:
        current_index = frame_counter[0]
        
        # Read data ONLY when the counter has incremented (New frame signal)
        if current_index > last_index:
            
            # --- START CRITICAL READ SECTION ---
            # Read the entire array from shared memory
            read_data = shared_array_view.copy() 
            
            # Process and Print Detections
            detection_count = 0
            output_string = ""
            
            # Iterate through potential detection slots (rows)
            for row in read_data:
                confidence = row[0]
                
                # Check if the confidence is greater than a minimum threshold (0.01)
                if confidence > 0.01: 
                    # Structure: [Confidence, xmin, ymin, xmax, ymax, Class_ID (float64)]
                    
                    xmin, ymin, xmax, ymax = row[1], row[2], row[3], row[4]
                    
                    output_string += (
                        f"  -> Conf: {confidence:.2f}, "
                        f"BBox: ({xmin:.2f}, {ymin:.2f}) to ({xmax:.2f}, {ymax:.2f})\n"
                    )
                    detection_count += 1
            # --- END CRITICAL READ SECTION ---

            # Update status
            frames_read += 1
            last_index = current_index
            
            # Display result
            sys.stdout.write(f"\n[{frames_read:05d}] HOST FRAME {current_index} - Read {detection_count} Detections:\n")
            sys.stdout.write(output_string)
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
    # Release Resources (The reader only closes its view)
    print("\n--- Container: Releasing resources ---")
    if shm_data:
        shm_data.close()
    if shm_state:
        shm_state.close()
    print("✅ SHM view released successfully.")
    sys.exit(0)
