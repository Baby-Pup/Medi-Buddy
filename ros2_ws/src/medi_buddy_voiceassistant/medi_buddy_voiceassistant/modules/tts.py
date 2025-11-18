import edge_tts
import asyncio
import sounddevice as sd
import soundfile as sf


class TTS:
    def __init__(self, voice="ko-KR-SunHiNeural", output_file="output.wav"):
        """
        EdgeTTS 클래스 초기화
        
        Args:
            voice (str): TTS 음성 선택
            output_file (str): 출력 파일 경로
        """
        self.voice = voice
        self.output_file = output_file
    
    async def _make_tts_async(self, text):
        """비동기로 TTS 생성"""
        communicate = edge_tts.Communicate(text=text, voice=self.voice)
        await communicate.save(self.output_file)
    
    def make_tts(self, text):
        """
        텍스트를 음성으로 변환
        
        Args:
            text (str): 변환할 텍스트
        """
        try:
            asyncio.run(self._make_tts_async(text))
            print(f"TTS 생성 완료: {self.output_file}")
        except Exception as e:
            print(f"TTS 생성 실패: {e}")
    
    def play_audio(self, file_path=None):
        """
        오디오 재생
        
        Args:
            file_path (str): 재생할 파일 경로
        """
        if file_path is None:
            file_path = self.output_file
        
        try:
            data, sample_rate = sf.read(file_path)
            sd.play(data, sample_rate)
            sd.wait()
            print(f"재생 완료")
        except Exception as e:
            print(f"재생 실패: {e}")
    
    def make_and_play(self, text):
        """텍스트를 음성으로 변환하고 재생"""
        self.make_tts(text)
        self.play_audio()


if __name__ == "__main__":
    tts = TTS()
    tts.make_and_play("메인 실행")