import cv2
import face_recognition
import numpy as np


class FaceDetection:
    def __init__(self, name='temp', tolerance=0.3, target_detect=None, headless=False):
        """
        얼굴 인식 클래스 초기화
        """
        self.tolerance = tolerance
        self.name = name
        self.target_detect = target_detect
        self.target_detected_flag = False
        self.headless = headless
        
        # ROS 노드에서 인코딩을 완료하면 여기에 저장됩니다.
        self.known_face = None 
        

    def _get_biggest_face(self, face_locations):
        """
        얼굴 위치 리스트에서 가장 큰 얼굴 반환
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


    # 💡 ROS 구독 기반 인코딩을 위해 수정된 함수
    def encode_from_frame(self, frame):
        """
        단일 OpenCV 프레임에서 가장 큰 얼굴을 찾아 인코딩을 반환.
        (ROS 구독 기반 인코딩을 위해 DetSubscriber에서 호출됨)
        """
        # 1. RGB로 변환
        rgb_frame = np.ascontiguousarray(frame[:, :, ::-1])
        
        # 2. 얼굴 위치 찾기
        face_locations = face_recognition.face_locations(rgb_frame)
        biggest_face = self._get_biggest_face(face_locations)
        
        if biggest_face is not None:
            # 3. 가장 큰 얼굴만 잘라 인코딩
            top, right, bottom, left = biggest_face
            face_image = frame[top:bottom, left:right]
            face_image_rgb = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
            encodings = face_recognition.face_encodings(face_image_rgb)
            
            if encodings:
                return encodings[0]
        
        return None
        
    # 🚨 기존의 face_encoding() 및 face_detection() 함수는 ROS 구독자 모델에 맞지 않으므로 제거됩니다.
    #    (또는 제거 대신 주석 처리 및 이름을 변경하여 사용하지 않도록 합니다.)


    def process_frame(self, frame):
        """
        ROS 콜백에서 받은 단일 프레임을 처리하여 얼굴 감지를 수행.
        """
        if self.known_face is None:
            # 인코딩이 되어있지 않으면 무시
            return
            
        # 1. RGB로 변환
        rgb_frame = np.ascontiguousarray(frame[:, :, ::-1])
        
        # 2. 모든 얼굴 위치와 인코딩 찾기
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        
        # 3. 타겟 발견 여부 확인
        target_found_in_frame = False
        
        for face_encoding in face_encodings:
            # 알려진 얼굴과 비교
            matches = face_recognition.compare_faces([self.known_face], face_encoding, 
                                                    tolerance=self.tolerance)
            
            if matches[0]:
                target_found_in_frame = True
                break 
        
        # 4. 타겟 발견 상태 토글 및 콜백 호출
        if target_found_in_frame and not self.target_detected_flag:
            self.target_detected_flag = True
            if self.target_detect is not None:
                self.target_detect(True, self.name)
            
        elif not target_found_in_frame and self.target_detected_flag:
            self.target_detected_flag = False
            if self.target_detect is not None:
                self.target_detect(False, self.name)
