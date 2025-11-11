from openai import OpenAI
from modules.sst import RMS_VAD
from modules.tts import TTS
import os

MY_API_KEY = os.environ.get("OPENAPI_KEY")
llm_model = OpenAI(api_key=MY_API_KEY).chat.completions
rv = RMS_VAD()
tts = TTS()

for _ in range(3):
    result = rv.run()
    re = llm_model.create(model='gpt-3.5-turbo', messages=[
    {"role": "system",
        "content": "당신은 의료 지원 로봇 'Medi-Buddy'입니다. 3문장 내외 존댓말로 친근하게 답변합니다.",},
    {"role": "user", "content": result}]).choices[0].message.content
    print(re)
    tts.make_and_play(re)
