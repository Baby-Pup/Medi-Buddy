 
# Face Recognition System
실시간 얼굴 인식 시스템입니다. 특정 타겟의 얼굴을 등록하고, 웹캠을 통해 실시간으로 감지하여 콜백 함수를 호출합니다.

## About
- **face_detection.py**: 얼굴 인코딩 생성 및 실시간 감지 기능 제공

## Features
- 실시간 다중 얼굴 감지 (Unknown 자동 표시)
- 타겟 발견/사라짐 이벤트 콜백
- Headless 모드 지원 (라즈베리파이 최적화)
- 가장 큰 얼굴 자동 선택
- 신뢰도(confidence) 표시

## How to Start

### 1. 리포지토리 클론
```bash
git clone https://github.com/Che-Serene/Medi-Buddy-FaceDetection.git
cd Medi-Buddy-FaceDetection
```
 

### 2. 가상환경 실행
```bash
python3 -m venv .venv
source .venv/bin/activate
```
 

### 3. 필요한 패키지 설치
```bash
pip install -r requirements.txt
```
 

## 예시 코드

### 기본 사용 (GUI 모드)
```python
from modules.face_detection import FaceDetection

fd = FaceDetection(name='temp', tolerance=0.3, headless=False)
encoding = fd.face_encoding()

if encoding is not None:
  fd.face_detection(encoding)
```
 

### Headless 모드 (라즈베리파이)
```python
from modules.face_detection import FaceDetection

def on_target_detected(detected, name):
  if detected:
    print(f"★★★ {name}님이 감지되었습니다! ★★★")
    # 원하는 동작 추가 (출발 신호 전송)
  else:
    print(f"☆☆☆ {name}님이 화면에서 사라졌습니다. ☆☆☆")
    # 원하는 동작 추가 (멈춤)

fd = FaceDetection(
  name='temp',
  tolerance=0.3,
  target_detect=on_target_detected,
  headless=True
  )

encoding = fd.face_encoding()

if encoding is not None:
  fd.face_detection(encoding)
```
 

### 얼굴 인코딩 저장 및 재사용
```python
from modules.face_detection import FaceDetection
import numpy as np

# 1. 얼굴 등록 및 저장
fd = FaceDetection(name='YourName', tolerance=0.3, headless=False)
encoding = fd.face_encoding()

if encoding is not None:
  np.save('face_encoding.npy', encoding)
  print("얼굴 인코딩 저장 완료!")

# 2. 저장된 인코딩 로드 및 사용
encoding = np.load('face_encoding.npy')
  fd = FaceDetection(name='YourName', tolerance=0.3, headless=True)
  fd.face_detection(encoding)
```


## Parameters
- `name` (str, default='temp'): 등록할 사람의 이름
- `tolerance` (float, default=0.3): 얼굴 비교 임계값 (0.3 ~ 0.4 권장, 낮을수록 엄격)
- `target_detect` (callable, optional): 타겟 발견/사라짐 시 호출할 콜백 함수
- `headless` (bool, default=False): GUI 없이 실행 여부 (라즈베리파이는 True 권장)
