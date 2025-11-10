# src/vision_module.py
# -----------------------------------------------------------
# YOLO 모델과 카메라 초기화, 예측 기능을 클래스화
# -----------------------------------------------------------

import cv2
import yaml
from ultralytics import YOLO

class VisionModule:
    """카메라 + YOLO 모델 관리 클래스"""
    def __init__(self, config):
        """
        config 객체(C.MODEL_PATH_DIR, C.YAML_PATH_DIR, C.CAM_INDEX)를 이용해 초기화
        """
        self.config = config
        self.model = None
        self.class_names = []
        self.width = None
        self.height = None

        self._load_model()
        self._init_camera()

    # -------------------------------------------------------
    # 내부 함수: 모델 로드
    # -------------------------------------------------------
    def _load_model(self):
        try:
            with open(self.config.YAML_PATH_DIR, 'r') as f:
                data_yaml = yaml.safe_load(f)
                self.class_names = data_yaml.get('names', [])
                print(f"[INFO] YOLO 클래스 이름 로드: {self.class_names}")

            self.model = YOLO(self.config.MODEL_PATH_DIR)
            print("[INFO] YOLO 모델 로드 성공.")

        except FileNotFoundError:
            print(f"[ERROR] YAML 파일을 찾을 수 없습니다: {self.config.YAML_PATH_DIR}")
        except Exception as e:
            print(f"[ERROR] YOLO 모델 로드 실패: {e}")

    # -------------------------------------------------------
    # YOLO 예측 수행
    # -------------------------------------------------------
    def detect(self, frame):
        """
        입력 프레임을 YOLO로 분석하고
        (detected_type, class_name)을 반환
        """
        if self.model is None:
            return "unknown", None

        results = self.model(frame, device='cpu', verbose=False)
        boxes = results[0].boxes
        if len(boxes) == 0:
            return "unknown", None

        cls_idx = int(boxes[0].cls[0])
        class_name = self.class_names[cls_idx] if cls_idx < len(self.class_names) else "unknown"

        # anomaly 계열이면 anomaly, normal이면 normal로 분류
        if class_name.startswith("anomaly"):
            detected_type = "anomaly"
        elif class_name == "normal":
            detected_type = "normal"
        else:
            detected_type = "unknown"

        return detected_type, class_name
