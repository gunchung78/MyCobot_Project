#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from ultralytics import YOLO
from src.config_loader import load_config 
import src.utility as util
from src.detector import PickTargetDetector
from src.vision_module import VisionModule 
from src.robot import Robot
from src.ros_robot import *
import argparse

def main():
    # ---- 인자 설정 ----
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", type=str, default="detect_and_classify")
    args = ap.parse_args()
    
    # ---- 설정 로드 ----
    C = load_config("config.json")

    # ---- YOLO 로드 ----
    if args.mode == "detect_and_classify": 
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
            if args.mode == "detect_and_classify":
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

                bot = ROS_Robot(args.mode, C, x_t, y_t, rz_t, color, detected_type)
                out = bot.main_fow()
                for step in out:
                    if step[0] == "graps":
                        print("graps")
                    else:
                        r.move_and_wait(step[0],step[1],C.MOVE_SPEED)
                print(out)
            key = cv2.waitKey(3) & 0xFF
            if key == 27:  # ESC
                break

    finally:
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
