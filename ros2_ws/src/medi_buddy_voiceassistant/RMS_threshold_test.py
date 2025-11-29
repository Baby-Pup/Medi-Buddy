import pyaudio
import numpy as np

sample_rate = 16000
frame_duration = 30  # ms
frame_size = int(sample_rate * frame_duration / 1000)  # 프레임 길이 샘플 수

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=sample_rate,
                input=True,
                frames_per_buffer=frame_size)

print("마이크 입력 RMS 값 측정 시작. Ctrl+C로 종료하세요.")

try:
    while True:
        frame = stream.read(frame_size, exception_on_overflow=False)
        pcm_data = np.frombuffer(frame, dtype=np.int16)
        rms = np.sqrt(np.mean(pcm_data.astype(np.float32)**2))
        print(f"RMS 값: {rms:.2f}")
except KeyboardInterrupt:
    print("측정 종료")

stream.stop_stream()
stream.close()
p.terminate()
