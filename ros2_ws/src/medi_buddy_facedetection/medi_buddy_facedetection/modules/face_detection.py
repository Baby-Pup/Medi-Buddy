import cv2
import face_recognition
import numpy as np


class FaceDetection:
    def __init__(self, name='temp', tolerance=0.3, target_detect=None, headless=False):
        """
        얼굴 인식 클래스 초기화
        
        Args:
            name: 등록할 사람의 이름 (문자열)
            tolerance: 얼굴 비교 임계값 (낮을수록 엄격, 0.3 ~ 0.4 권장)
            target_detect: 타겟 발견 시 호출할 콜백 함수 (선택사항)
            headless: True면 GUI 없이 실행 (라즈베리파이 최적화)
        """
        self.tolerance = tolerance
        self.name = name
        self.target_detect = target_detect
        self.target_detected_flag = False
        self.headless = headless
        

    def _get_biggest_face(self, face_locations):
        """
        얼굴 위치 리스트에서 가장 큰 얼굴 반환
        
        Args:
            face_locations: face_recognition.face_locations()의 결과
            
        Returns:
            가장 큰 얼굴의 위치 (top, right, bottom, left)
        """
        if not face_locations:
            return None
            
        max_area = 0
        biggest_face = None
        
        for face_loc in face_locations:
            top, right, bottom, left = face_loc
            area = (bottom - top) * (right - left)
            
            if area > max_area:
                max_area = area
                biggest_face = face_loc
                
        return biggest_face


    def face_encoding(self):
        """
        웹캠으로 얼굴을 촬영하고 인코딩 반환
        's' 키를 누르면 가장 큰 얼굴을 저장
        'q' 키를 누르면 종료
        headless 모드에서는 3초 후 자동으로 가장 큰 얼굴 저장
        
        Returns:
            얼굴 인코딩 (numpy array)
        """
        video_capture = cv2.VideoCapture(0)
        
        if not video_capture.isOpened():
            print("카메라를 열 수 없습니다.")
            return None
        
        if self.headless:
            print("Headless 모드: 3초 후 자동으로 얼굴 저장...")
            frame_count = 0
            target_frames = 90  # 30fps 기준 3초
            
            while frame_count < target_frames:
                ret, frame = video_capture.read()
                if not ret:
                    print("프레임을 읽을 수 없습니다.")
                    break
                    
                frame_count += 1
                
                # 마지막 프레임에서 얼굴 인코딩
                if frame_count >= target_frames:
                    rgb_frame = np.ascontiguousarray(frame[:, :, ::-1])
                    face_locations = face_recognition.face_locations(rgb_frame)
                    biggest_face = self._get_biggest_face(face_locations)
                    
                    if biggest_face is not None:
                        top, right, bottom, left = biggest_face
                        face_image = frame[top:bottom, left:right]
                        face_image_rgb = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
                        encodings = face_recognition.face_encodings(face_image_rgb)
                        
                        if encodings:
                            print("얼굴 인코딩 완료!")
                            video_capture.release()
                            return encodings[0]
                    
                    print("얼굴을 찾을 수 없습니다.")
                    break
            
            video_capture.release()
            return None
        
        else:
            # GUI 모드
            print("'s' 키를 눌러 가장 큰 얼굴 저장, 'q' 키로 종료")
            
            while True:
                ret, frame = video_capture.read()
                
                if not ret:
                    print("프레임을 읽을 수 없습니다.")
                    break
                
                rgb_frame = np.ascontiguousarray(frame[:, :, ::-1])
                face_locations = face_recognition.face_locations(rgb_frame)
                display_frame = frame.copy()
                biggest_face = self._get_biggest_face(face_locations)
                
                for top, right, bottom, left in face_locations:
                    color = (0, 255, 0) if (top, right, bottom, left) == biggest_face else (128, 128, 128)
                    cv2.rectangle(display_frame, (left, top), (right, bottom), color, 2)
                    
                    if (top, right, bottom, left) == biggest_face:
                        cv2.putText(display_frame, "Biggest Face", (left, top - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.imshow('Video', display_frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                
                if key == ord('s') and biggest_face is not None:
                    top, right, bottom, left = biggest_face
                    face_image = frame[top:bottom, left:right]
                    face_image_rgb = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
                    encodings = face_recognition.face_encodings(face_image_rgb)
                    
                    if encodings:
                        print("얼굴 인코딩 완료!")
                        video_capture.release()
                        cv2.destroyAllWindows()
                        return encodings[0]
                    else:
                        print("얼굴 인코딩 실패. 다시 시도하세요.")
            
            video_capture.release()
            cv2.destroyAllWindows()
            return None
            

    def face_detection(self, known_face):
        """
        실시간으로 여러 얼굴 인식 수행
        등록된 타겟이 발견되면 콜백 함수 호출
        headless 모드에서는 화면 표시 없이 백그라운드 실행
        
        Args:
            known_face: 등록된 얼굴 인코딩 (numpy array)
        """
        if known_face is None:
            print("등록된 얼굴이 없습니다.")
            return
        
        video_capture = cv2.VideoCapture(0)
        
        if not video_capture.isOpened():
            print("카메라를 열 수 없습니다.")
            return
        
        # 처리 속도 향상을 위한 프레임 스킵 카운터
        frame_count = 0
        process_every_n_frames = 3 if self.headless else 2  # headless에서 더 많이 스킵
        
        print(f"얼굴 인식 시작 ({'Headless' if self.headless else 'GUI'} 모드)")
        if not self.headless:
            print("'q' 키로 종료")
        else:
            print("Ctrl+C로 종료")
        
        try:
            while True:
                ret, frame = video_capture.read()
                
                if not ret:
                    break
                
                frame_count += 1
                
                # 매 N 프레임마다만 얼굴 인식 수행
                if frame_count % process_every_n_frames == 0:
                    # RGB로 변환
                    rgb_frame = np.ascontiguousarray(frame[:, :, ::-1])
                    
                    # 모든 얼굴 위치와 인코딩 찾기
                    face_locations = face_recognition.face_locations(rgb_frame)
                    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
                    
                    # 타겟 발견 여부
                    target_found_in_frame = False
                    
                    # Headless 모드에서는 화면 렌더링 스킵
                    if not self.headless:
                        display_frame = frame.copy()
                    
                    # 찾은 모든 얼굴 처리
                    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                        # 알려진 얼굴과 비교
                        matches = face_recognition.compare_faces([known_face], face_encoding, 
                                                                tolerance=self.tolerance)
                        name = "Unknown"
                        confidence = 0
                        
                        # 얼굴 거리 계산
                        face_distance = face_recognition.face_distance([known_face], face_encoding)[0]
                        
                        if matches[0]:
                            name = self.name
                            confidence = (1 - face_distance) * 100
                            target_found_in_frame = True
                        
                        # GUI 모드에서만 화면에 그리기
                        if not self.headless:
                            color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                            cv2.rectangle(display_frame, (left, top), (right, bottom), color, 2)
                            cv2.rectangle(display_frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                            font = cv2.FONT_HERSHEY_DUPLEX
                            text = f"{name} ({confidence:.1f}%)" if name != "Unknown" else name
                            cv2.putText(display_frame, text, (left + 6, bottom - 6), 
                                      font, 0.6, (255, 255, 255), 1)
                    
                    # 타겟 발견 상태 토글 및 콜백 호출
                    if target_found_in_frame and not self.target_detected_flag:
                        self.target_detected_flag = True
                        if self.target_detect is not None:
                            self.target_detect(True, self.name)
                        print(f"[타겟 발견] {self.name}")
                        
                    elif not target_found_in_frame and self.target_detected_flag:
                        self.target_detected_flag = False
                        if self.target_detect is not None:
                            self.target_detect(False, self.name)
                        print(f"[타겟 사라짐] {self.name}")
                    
                    # GUI 모드에서만 화면 표시
                    if not self.headless:
                        cv2.imshow('Video', display_frame)
                
                # GUI 모드에서만 키 입력 처리
                if not self.headless:
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            print("\n프로그램 종료")
        finally:
            video_capture.release()
            if not self.headless:
                cv2.destroyAllWindows()


# 콜백 함수 예제
def on_target_detected(detected, name):
    """
    타겟 발견/사라짐 시 호출되는 콜백 함수
    
    Args:
        detected: True면 타겟 발견, False면 타겟 사라짐
        name: 타겟의 이름
    """
    if detected:
        print(f"★★★ {name}님이 감지되었습니다! ★★★")
        # 원하는 동작 추가 (출발 신호 전송)
    else:
        print(f"☆☆☆ {name}님이 화면에서 사라졌습니다. ☆☆☆")
        # 원하는 동작 추가 (멈춤)


if __name__ == "__main__":
    # Headless 모드로 실행 (라즈베리파이 최적화)
    fd = FaceDetection(name='temp', tolerance=0.3, target_detect=on_target_detected, headless=True)
    
    # 또는 GUI 모드로 실행
    # fd = FaceDetection(name='temp', tolerance=0.3, target_detect=on_target_detected, headless=False)
    
    encoding = fd.face_encoding()
    
    if encoding is not None:
        fd.face_detection(encoding)
    else:
        print("얼굴 인코딩에 실패했습니다.")
