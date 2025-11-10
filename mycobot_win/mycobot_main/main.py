#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import yaml
from ultralytics import YOLO
from src.config_loader import load_config 
import src.utility as util
from src.detector import PickTargetDetector
from src.robot import Robot

# YOLO 설정
# MODEL_PATH = 'C:/Dev/KAIROS_Project_2/AI_Cordinate/YOLO/runs/detect/train/weights/best.pt'
MODEL_PATH = '/home/jy/MyCobot_Project/mycobot_win/mycobot_vision/best.pt'
# YAML_PATH = 'C:/Dev/KAIROS_Project_2/AI_Cordinate/YOLO/team/custom_data.yaml'
YAML_PATH = '/home/jy/MyCobot_Project/mycobot_win/mycobot_vision/custom_data.yaml'
# YAML 로드
try:
    with open(YAML_PATH, 'r') as f:
        data_yaml = yaml.safe_load(f)
        CLASS_NAMES = data_yaml['names']
        print(f"[INFO] YOLO 클래스 이름 로드: {CLASS_NAMES}")
except FileNotFoundError:
    print(f"[ERROR] YAML 파일을 찾을 수 없습니다: {YAML_PATH}")
    CLASS_NAMES = []

# YOLO 모델 로드
try:
    model = YOLO(MODEL_PATH)
    print("[INFO] YOLO 모델 로드 성공.")
except Exception as e:
    print(f"[ERROR] YOLO 모델 로드 실패: {e}")
    model = None

def main():
    # ---- 설정 로드 ----
    C = load_config("config.json")

    # ---- 카메라 준비 ----
    cam = cv2.VideoCapture(C.CAM_INDEX)
    if not cam.isOpened():
        raise SystemExit("카메라를 열 수 없습니다.")
    ok, frame0 = cam.read()
    if not ok:
        cam.release()
        raise SystemExit("첫 프레임을 읽지 못했습니다.")
    H, W = frame0.shape[:2]

    # ---- Detector 준비 ----
    detector = PickTargetDetector(C, util)
    detector.set_frame_size(W, H)

    # ---- 로봇 준비(루프 외부에서 1회만) ----
    r = Robot(C.PORT, C.BAUD, C.MOVE_SPEED, C.GRIPPER_OPEN_VAL, C.GRIPPER_CLOSE_VAL, C.GRIPPER_SPEED, C.ANCHOR_PY)
    r.power_on()
    r.gripper_open()
    placeList=[0,0,0]   # red, blue, green

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("[ERROR] 프레임 읽기 실패.")
                break
                
            # [추가] YOLO 모델을 통한 불량/정상 판별
            detected_type = None
            if model is not None:
                results = model(frame, device='cpu', verbose=False)
                boxes = results[0].boxes
                if len(boxes) > 0:
                    cls_idx = int(boxes[0].cls[0])  # 첫 번째 객체 클래스 인덱스
                    class_name = CLASS_NAMES[cls_idx]
                    print(f"[YOLO DETECT] 검출된 클래스: {class_name}")

                    # anomaly 계열이면 anomaly, normal이면 normal로 분류
                    if class_name.startswith("anomaly"):
                        detected_type = "anomaly"
                    elif class_name == "normal":
                        detected_type = "normal"
                    else:
                        detected_type = "unknown"

            # ---- 기존 detector (색상 판단) ----
            result = detector.process(frame)
            output = result["overlay"]

            if result["has_target"] and result["robot_action"]:
                x_t = result["x_t"]
                y_t = result["y_t"]
                rz_t = result["rz_t"] if result["rz_t"] is not None else C.ANCHOR_PY[5]
                color = result["color"]

                print(f"[INFO] 감지 색상: {color}, YOLO 판정: {detected_type}")

                try:
                    # 접근(상부)
                    r.move_coords([x_t, y_t, C.ANCHOR_PY[2]+30, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                    r.move_coords([x_t, y_t, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                    # 집기
                    r.gripper_close()
                    # 복귀
                    r.move_coords(C.ANCHOR_PY, C.MOVE_SPEED, 0, sleep=0.5)

                    # [변경된 분류 로직]
                    # 색상 + YOLO 결과 모두 고려
                    if color == "red":
                        print("[ACTION] red 감지 → anomaly 위치로 이동")
                        r.place_box("anomaly", placeList[0])
                        placeList[0] += 1
                    elif color == "green":
                        print("[ACTION] green 감지 → normal 위치로 이동")
                        r.place_box("normal", placeList[2])
                        placeList[2] += 1
                    elif color == "blue":
                        print("[ACTION] blue 감지 → blue 위치로 이동")
                        r.place_box("blue", placeList[1])
                        placeList[1] += 1
                    # 추가로 YOLO 결과에 따라 색상 불분명시 대체 동작 수행
                    elif detected_type == "anomaly":
                        print("[ACTION] YOLO anomaly 판정 → anomaly 위치로 이동")
                        r.place_box("anomaly", 2)
                    elif detected_type == "normal":
                        print("[ACTION] YOLO normal 판정 → normal 위치로 이동")
                        r.place_box("normal", 2)
                    else:
                        print("[WARN] 색상 및 YOLO 판정 불명 → 동작 생략")

                except Exception as e:
                    print(f"[ERROR] 로봇 명령 실패: {e}")

            # 시각화/키 처리
            cv2.imshow("Frame", output)
            key = cv2.waitKey(3) & 0xFF
            if key == 27:  # ESC
                break

    finally:
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
