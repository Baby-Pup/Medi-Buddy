#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyaudio
import collections
import numpy as np
from scipy.signal import resample
from pydub import AudioSegment
import base64
import time


class AudioRecorderNode(Node):
    def __init__(self):
        super().__init__('audio_recorder_node')

        # 음성 파일(mp3 base64) 퍼블리셔
        self.audio_pub = self.create_publisher(String, '/recorded_audio_mp3', 10)
        # robot_status 퍼블리셔
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # ------------------------------
        # 녹음 설정
        # ------------------------------
        self.mic_rate = 48000
        self.sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.mic_rate * self.frame_duration / 1000)

        self.threshold = 2000  # RMS threshold

        self.get_logger().info("🎤 Audio Recorder Node Started")

    def is_loud_enough(self, frame):
        """RMS 기반 음성 감지"""
        pcm = np.frombuffer(frame, dtype=np.int16)
        if pcm.size == 0:
            return False

        rms = np.sqrt(np.mean(pcm.astype(np.float32) ** 2))
        return rms > self.threshold

    def record_voice(self):
        """음성을 감지해서 녹음하고 mp3로 저장 후 base64 인코딩 반환"""

        p = pyaudio.PyAudio()

        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.mic_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

        ring_buffer = collections.deque(maxlen=10)
        voiced_frames = []
        triggered = False

        self.get_logger().info("⏳ 대기 중... 말하면 자동으로 녹음 시작합니다.")

        while True:
            frame = stream.read(self.frame_size, exception_on_overflow=False)
            is_speech = self.is_loud_enough(frame)

            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])

                if num_voiced > 0.7 * ring_buffer.maxlen:
                    triggered = True
                    self.get_logger().info("🎙️ 음성 감지 → 녹음 시작")

                    # 음성 처리 시작 상태 전송
                    status_msg = String()
                    status_msg.data = "audio_incoming"
                    self.status_pub.publish(status_msg)
                    self.get_logger().info("📡 상태 전송: audio_incoming")
                    
                    voiced_frames.extend([f for f, s in ring_buffer])
                    ring_buffer.clear()

            else:
                voiced_frames.append(frame)
                ring_buffer.append((frame, is_speech))

                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > 0.7 * ring_buffer.maxlen:
                    self.get_logger().info("🛑 음성 종료 → 녹음 중지")
                    break

        stream.stop_stream()
        stream.close()
        p.terminate()

        # -----------------------------
        # 48000Hz → 16000Hz 리샘플링
        # -----------------------------
        audio_data = b"".join(voiced_frames)
        audio_np = np.frombuffer(audio_data, dtype=np.int16)

        new_length = int(len(audio_np) * (self.sample_rate / self.mic_rate))
        resampled_audio = resample(audio_np, new_length).astype(np.int16)

        # -----------------------------
        # WAV 형식으로 변환
        # -----------------------------
        wav_audio = AudioSegment(
            resampled_audio.tobytes(),
            frame_rate=16000,
            sample_width=2,
            channels=1
        )

        # -----------------------------
        # MP3로 저장
        # -----------------------------
        filename = f"/tmp/voice.mp3"
        wav_audio.export(filename, format="mp3")

        self.get_logger().info(f"💾 MP3 저장 완료: {filename}")

        # -----------------------------
        # Base64 인코딩
        # -----------------------------
        with open(filename, "rb") as f:
            encoded_mp3 = base64.b64encode(f.read()).decode("utf-8")

        return encoded_mp3

    def run(self):
        """녹음 → mp3 인코딩 → ROS 퍼블리시"""

        encoded_mp3 = self.record_voice()

        msg = String()
        msg.data = encoded_mp3

        self.audio_pub.publish(msg)
        self.get_logger().info("📡 MP3 파일 Base64 퍼블리시 완료")

        self.get_logger().info("⏳ 다음 음성을 기다립니다...")


def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorderNode()

    try:
        while rclpy.ok():           # ← 반복 녹음 모드
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
