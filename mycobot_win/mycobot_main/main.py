#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from ultralytics import YOLO
from src.config_loader import load_config 
import src.utility as util
from src.detector import PickTargetDetector
from src.vision_module import VisionModule 
from src.robot import Robot
import argparse

def main():
    # ---- 인자 설정 ----
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", type=int, default=0)
    args = ap.parse_args()
    
    # ---- 설정 로드 ----
    C = load_config("config.json")

    # ---- YOLO 로드 ----
    if args.mode == 0: 
        vision = VisionModule(C)

    # ---- 카메라 준비 ----
    cam = cv2.VideoCapture(C.CAM_INDEX)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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

    def read_latest(cap, discard=4):
    # discard번 grab만 하고, 마지막 한 장만 retrieve
        for _ in range(discard):
            cap.grab()
        ret, frame = cap.retrieve()
        return ret, frame

    try:
        while True:
            ret, frame = read_latest(cam, discard=4)
            if not ret:
                print("[ERROR] 프레임 읽기 실패.")
                break
                
            # ---- YOLO normal or anomaly ----
            if args.mode == 0:
                detected_type, class_name = vision.detect(frame)
                if(class_name is not None):
                    print(f"[YOLO] Detected: {class_name} ({detected_type})")
            else:
                detected_type = None
                class_name = None

            # ---- 기존 detector (색상 판단) ----
            result = detector.process(frame)
            output = result["overlay"]

            # ---- 시각화 처리 ----
            cv2.imshow("Frame", output)

            if result["has_target"] and result["robot_action"]:
                x_t = result["x_t"]
                y_t = result["y_t"]
                rz_t = result["rz_t"] if result["rz_t"] is not None else C.ANCHOR_PY[5]
                color = result["color"]
                print(f"[INFO] 감지 색상: {color}, YOLO 판정: {detected_type}")

                try:
                    if args.mode == 1 and (color == "white" or color is None ):
                       print(f'no color {color}')
                       r.refresh_home()
                    elif args.mode == 0 and (detected_type ==  "unknown" or detected_type is None): 
                       print(f'no detected_type {detected_type}')
                       r.refresh_home()

                    else:
                        # 접근(상부)
                        # r.move_coords([x_t, y_t, C.ANCHOR_PY[2]+30, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)

                        r.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

                        # r.move_coords([x_t, y_t, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                        r.move_and_wait("coords", [x_t, y_t, C.ANCHOR_PY[2], C.ANCHOR_PY[3], C.ANCHOR_PY[4] , rz_t], C.MOVE_SPEED, 0)

                        # 집기
                        r.gripper_close()
                        # 복귀
                        r.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

                        # [변경된 분류 로직]
                        # 색상 + YOLO 결과 모두 고려
                        if args.mode == 1:
                            if color == "red":
                                print("[ACTION] red 감지")
                                r.place_box("red", placeList[0])
                                placeList[0] += 1
                            elif color == "green":
                                print("[ACTION] green 감지")
                                r.place_box("green", placeList[2])
                                placeList[2] += 1
                            elif color == "blue":
                                print("[ACTION] blue 감지")
                                r.place_box("blue", placeList[1])
                                placeList[1] += 1
                        elif args.mode == 0: 
                            # 추가로 YOLO 결과에 따라 색상 불분명시 대체 동작 수행
                            if detected_type == "anomaly":
                                print("[ACTION] YOLO anomaly 판정")
                                r.place_box("anomaly")
                            elif detected_type == "normal":
                                print("[ACTION] YOLO normal 판정")
                                r.place_box("normal", placeList[2])
                                placeList[2] += 1
                            else:
                                print("[WARN] 색상 및 YOLO 판정 불명 → 동작 생략")
                except Exception as e:
                    print(f"[ERROR] 로봇 명령 실패: {e}")
            


            key = cv2.waitKey(3) & 0xFF
            if key == 27:  # ESC
                break

    finally:
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
