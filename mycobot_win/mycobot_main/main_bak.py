#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from src.config_loader import load_config 
import src.utility as util
from src.detector import PickTargetDetector
from src.robot import Robot
from src.vision_module import VisionModule   # ✅ [추가] VisionModule 클래스 import
import argparse

def main():
    # ---- 인자 설정 ----
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default=0)
    args = ap.parse_args()
    
    # ---- 설정 로드 ----
    C = load_config("config.json")

    # ✅ [변경] VisionModule 클래스로 카메라/모델 로드 통합
    vision = VisionModule(C)

    # ---- Detector 준비 ----
    detector = PickTargetDetector(C, util)
    detector.set_frame_size(vision.width, vision.height)

    # ---- 로봇 준비 ----
    r = Robot(C.PORT, C.BAUD, C.MOVE_SPEED, C.GRIPPER_OPEN_VAL, C.GRIPPER_CLOSE_VAL, C.GRIPPER_SPEED, C.ANCHOR_PY)
    r.power_on()
    r.gripper_open()
    placeList = [0, 0, 0]

    try:
        while True:
            frame = vision.get_frame()   # ✅ [변경] VisionModule에서 프레임 가져오기
            if frame is None:
                print("[ERROR] 프레임 읽기 실패.")
                break

            # ✅ [변경] YOLO 결과를 VisionModule에서 직접 호출
            detected_type, class_name = vision.detect(frame)
            print(f"[YOLO] Detected: {class_name} ({detected_type})")

            # ---- 기존 detector (색상 판단) ----
            result = detector.process(frame)
            output = result["overlay"]
            cv2.imshow("Frame", output)

            if result["has_target"] and result["robot_action"]:
                x_t = result["x_t"]
                y_t = result["y_t"]
                rz_t = result["rz_t"] if result["rz_t"] is not None else C.ANCHOR_PY[5]
                color = result["color"]

                print(f"[INFO] 감지 색상: {color}, YOLO 판정: {detected_type}")

                try:
                    r.move_coords([x_t, y_t, C.ANCHOR_PY[2]+30, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                    r.move_coords([x_t, y_t, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                    r.gripper_close()
                    r.move_coords(C.ANCHOR_PY, C.MOVE_SPEED, 0, sleep=0.5)

                    if args.mode == 1:
                        if color == "red":
                            print("[ACTION] red 감지 → anomaly 위치로 이동")
                            r.place_box("anomaly", placeList[0]); placeList[0] += 1
                        elif color == "green":
                            print("[ACTION] green 감지 → normal 위치로 이동")
                            r.place_box("normal", placeList[2]); placeList[2] += 1
                        elif color == "blue":
                            print("[ACTION] blue 감지 → blue 위치로 이동")
                            r.place_box("blue", placeList[1]); placeList[1] += 1
                    elif args.mode == 0:
                        if detected_type == "anomaly":
                            print("[ACTION] YOLO anomaly 판정 → anomaly 위치로 이동")
                            r.place_box("anomaly", 2)
                        elif detected_type == "normal":
                            print("[ACTION] YOLO normal 판정 → normal 위치로 이동")
                            r.place_box("normal", 2)
                        else:
                            print("[WARN] 색상 및 YOLO 판정 불명 → 동작 생략")

                except Exception as e:
                    print(f"[ERROR] 로봇 명령 실패: {e}")

            key = cv2.waitKey(3) & 0xFF
            if key == 27:
                break

    finally:
        vision.release()    # ✅ [변경] VisionModule에서 자원 해제
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
