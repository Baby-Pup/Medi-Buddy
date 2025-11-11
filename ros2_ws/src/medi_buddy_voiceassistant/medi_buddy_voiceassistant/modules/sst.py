import pyaudio
import webrtcvad
import collections
import numpy as np
from faster_whisper import WhisperModel
import sounddevice as sd
import time


class RMS_VAD:
    def __init__(self, model='tiny', threshold=5000):
        """
        RMS_VAD 클래스 초기화
        
        Args:
            model (str): Whisper 모델 크기 (tiny/small/base)
            threshold (int): 음성 감지 임계값
        """
        self.sample_rate = 16000
        frame_duration = 30
        self.frame_size = int(self.sample_rate * frame_duration / 1000)
        self.vad = webrtcvad.Vad(1)
        self.threshold = threshold
        self.term = 0
        self.model = model
        
        # 모델 로드
        try:
            self.sst_model = WhisperModel(model, compute_type='float32')
            print(f"모델 로드 완료: {model}")
        except Exception as e:
            print(f"모델 로드 실패: {e}")
            self.sst_model = None

    def is_loud_enough(self, frame):
        """
        음량이 충분한지 판단
        
        Args:
            frame (bytes): 오디오 프레임
            
        Returns:
            bool: 충분히 큰 음량이면 True
        """
        pcm_data = np.frombuffer(frame, dtype=np.int16)
        if pcm_data.size == 0:
            return False
        pcm_data = pcm_data.astype(np.float32)
        if np.any(np.isnan(pcm_data)) or np.any(np.isinf(pcm_data)):
            return False
        rms = np.sqrt(np.mean(pcm_data**2))
        return rms > self.threshold

    def listen_and_record(self):
        """
        마이크에서 음성을 감지하고 녹음
        
        Returns:
            np.ndarray: 정규화된 오디오 데이터
        """
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

        ring_buffer = collections.deque(maxlen=10)
        triggered = False
        voiced_frames = []

        print("계속 대기 중... 음성이 들리면 녹음 시작")

        while True:
            frame = stream.read(self.frame_size, exception_on_overflow=False)
            is_speech = self.vad.is_speech(frame, self.sample_rate) and self.is_loud_enough(frame)
            
            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])
                if num_voiced > 0.7 * ring_buffer.maxlen:
                    triggered = True
                    print("음성 감지: 녹음 시작")
                    voiced_frames.extend([f for f, s in ring_buffer])
                    ring_buffer.clear()
            else:
                voiced_frames.append(frame)
                ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > 0.7 * ring_buffer.maxlen:
                    print("음성 종료: 녹음 중지")
                    break

        stream.stop_stream()
        stream.close()
        p.terminate()

        audio_data = b"".join(voiced_frames)
        audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
        return audio_np

    def transcribe(self, audio):
        """
        오디오를 텍스트로 변환
        
        Args:
            audio (np.ndarray): 오디오 데이터
            
        Returns:
            tuple: (결과 텍스트, 추론 시간)
        """
        if self.sst_model is None:
            print("모델이 로드되지 않았습니다")
            return "", 0
        
        start_time = time.time()
        segments, info = self.sst_model.transcribe(audio, language='ko')
        result = ''.join([segment.text for segment in segments])
        inference_time = time.time() - start_time
        
        return result, inference_time

    def run(self):
        """
        음성 인식 실행
        
        Returns:
            result: 음성 인식 결과
        """
        audio = self.listen_and_record()
        result, inference_time = self.transcribe(audio)
        print(f"음성 인식 결과: {result}")
        print(f"SST 추론 시간: {inference_time:.2f}초")
        return result

if __name__ == "__main__":
    rv = RMS_VAD()
    rv.run()
