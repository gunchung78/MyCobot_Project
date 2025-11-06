#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from src.config_loader import load_config 
import src.utility as util
from src.detector import PickTargetDetector
from src.robot import Robot

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

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("[ERROR] 프레임 읽기 실패.")
                break
            
            result = detector.process(frame)
            output = result["overlay"]
            if result["has_target"] and result["robot_action"]:
                x_t = result["x_t"]
                y_t = result["y_t"]
                rz_t = result["rz_t"] if result["rz_t"] is not None else C.ANCHOR_PY[5]
                print(f"[RETURN] x={x_t:.2f}, y={y_t:.2f}, rz={rz_t:.2f} deg, pix=({result['u']},{result['v']}), color={result['color']}")
                try:
                    # 접근(상부)
                    r.move_coords([x_t, y_t, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                    # 집기
                    r.gripper_close()
                    # 복귀
                    r.move_coords(C.ANCHOR_PY, C.MOVE_SPEED, 0, sleep=0.5)
                    if True:
                        r.move_coords([x_t, 0, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], C.ANCHOR_PY[5]], C.MOVE_SPEED, 0, sleep=0.5)
                        r.gripper_open()
                        r.move_coords(C.ANCHOR_PY, C.MOVE_SPEED, 0, sleep=0.5)
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
