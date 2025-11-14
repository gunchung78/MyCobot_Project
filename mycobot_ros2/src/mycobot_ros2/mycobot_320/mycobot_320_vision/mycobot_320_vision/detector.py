#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from ultralytics import YOLO
import yaml
from pathlib import Path
import mycobot_320_vision.src.utility as util
from mycobot_320_vision.src.config_loader import load_config 
from mycobot_320_vision.src.ros_detector import PickTargetDetector

# ✅ config.json 로드
C = load_config("config.json")

def _safe_path(p):
    return Path(p).expanduser().resolve()

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.pub_result = self.create_publisher(Float32MultiArray, '/detector_result', 10)
        self.pub_classify = self.create_publisher(Int32, '/classify_result', 10)

        # --- 런치 파라미터로 모드 설정 (detect_only / detect_and_classify) ---
        self.declare_parameter('detect_mode', 'detect_only')
        self.mode = self.get_parameter('detect_mode').get_parameter_value().string_value
        self.get_logger().info(f"✅ DETECT MODE: {self.mode}")

        # --- YOLO 관련 변수 ---
        self.model = None
        self.class_names = []
        self.yaml_path = None
        self.model_path = None
        if self.mode == "detect_and_classify":
            self.yaml_path  = _safe_path(C.YAML_PATH_DIR)
            self.model_path = _safe_path(C.MODEL_PATH_DIR)
            self._init_yolo()
            
        # --- 내부 설정 초기화 ---
        self.robot_action = False
        self.has_target = False

        # --- 카메라 초기화 ---
        self.cap = cv2.VideoCapture(C.CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ 카메라를 열 수 없습니다. (CAM_INDEX={C.CAM_INDEX})")
            raise SystemExit
        ok, frame0 = self.cap.read()
        if not ok:
            raise SystemExit("첫 프레임을 읽지 못했습니다.")
        self.H, self.W = frame0.shape[:2]
        self.detector = PickTargetDetector(C, util)
        self.detector.set_frame_size(self.W, self.H)

        # --- 타이머 등록 ---
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("✅ DetectorNode initialized and running.")

    # ========== YOLO 초기화 ==========
    def _init_yolo(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data_yaml = yaml.safe_load(f)
                self.class_names = data_yaml.get('names', [])
                self.get_logger().info(f"[INFO] YOLO 클래스 이름 로드: {self.class_names}")
            self.model = YOLO(self.model_path)
            self.get_logger().info("[INFO] YOLO 모델 로드 성공.")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO 모델 로드 실패: {e}")

    # ========== YOLO 분류 수행 ==========    
    def classify_frame(self, frame):
        if self.model is None:
            return -1
        results = self.model(frame, device='cpu', verbose=False)  # Ultralytics YOLO 표준 추론 예시 참고, docs. :contentReference[oaicite:0]{index=0}
        boxes = results[0].boxes
        if len(boxes) == 0:
            return -1
        cls_idx = int(boxes[0].cls[0])
        if cls_idx >= len(self.class_names):
            return -1
        class_name = self.class_names[cls_idx]
        if class_name.startswith("anomaly"):
            return 1
        elif class_name == "normal":
            return 0
        else:
            return -1

    # ========== Main Loop ==========
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # detector
        try:
            result = self.detector.process(frame)
        except Exception as e:
            self.get_logger().error(f"process() failed: {e}")
            return
        output = result["overlay"]

        # /classify_result
        state = -1
        if self.mode == "detect_and_classify":
            state = self.classify_frame(frame)
            msg_state = Int32()
            msg_state.data = state
            self.pub_classify.publish(msg_state)

            state_txt = "ANOMALY" if state == 1 else ("NORMAL" if state == 0 else "UNKNOWN")
            cv2.putText(output, f"STATE: {state_txt}", (self.W - 220, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 200, 255) if state == 1 else (0, 220, 0) if state == 0 else (200, 200, 200), 2)

        # /detector_result
        self.has_target = result["has_target"]
        self.robot_action = result["robot_action"]
        if self.has_target and self.robot_action:
            x_t = float(result["x_t"])
            y_t = float(result["y_t"])
            rz_t = float(result["rz_t"]) if (result["rz_t"] is not None) else float(C.ANCHOR_PY[5])
            color = result["color"]

            # 색상코드 매핑
            color_map = {"red": 1.0, "blue": 2.0, "green": 3.0}
            color_code = color_map.get(color, 0.0)

            msg = Float32MultiArray()
            msg.data = [x_t, y_t, rz_t, color_code, -1.0]  # detected_code는 -1 유지(설계 그대로)
            self.pub_result.publish(msg)

        # 오버레이 화면 출력
        cv2.imshow("Frame", output)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧩 Shutting down detector.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()