#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
실시간 카메라에서 흰색/초록색 물체를 감지하고 중심 좌표(u, v)와 (옵션) Roll 각도를 계산/표시합니다.
- Windows에서는 CAP_DSHOW 백엔드로 안정성을 높입니다.
- 기본 색상 범위: HSV 기준으로 초록/흰색
- 면적이 일정 기준(기본 1000px^2) 이상인 가장 큰 컨투어를 대상으로 계산합니다.

사용법 예시:
  python live_box_detector.py                # 자동으로 카메라 인덱스를 탐색하고 실행
  python live_box_detector.py --index 1      # 특정 카메라 인덱스로 실행
  python live_box_detector.py --no-roll      # Roll 각도 계산/표시 끄기
  python live_box_detector.py --min-area 800 # 최소 면적 기준 변경
"""

import argparse
import sys
import time
import cv2
import numpy as np
import platform


def _open_capture(index: int) -> cv2.VideoCapture:
    """플랫폼에 따라 적절한 백엔드로 VideoCapture를 생성합니다."""
    if platform.system().lower().startswith("win"):
        return cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        return cv2.VideoCapture(index)


def find_camera_index(max_try: int = 3) -> list:
    """
    시스템에 연결된 유효한 카메라 인덱스를 찾아 리스트로 반환합니다.
    """
    print("--- 📸 사용 가능한 카메라 인덱스 테스트 중 ---")
    available_indices = []

    for i in range(max_try):
        cap = _open_capture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"✅ 인덱스 {i}: 성공적으로 카메라를 열고 프레임 읽기 성공!")
                available_indices.append(i)
            else:
                print(f"⚠️ 인덱스 {i}: 카메라를 열었으나 프레임 읽기 실패.")
            cap.release()
        else:
            print(f"❌ 인덱스 {i}: 카메라 열기 실패. 이 인덱스에는 장치가 없습니다.")

    if not available_indices:
        print("\n**오류:** 시스템에서 유효한 카메라 장치를 찾지 못했습니다. 드라이버/연결 상태를 확인하세요.")
    else:
        print(f"\n✅ 유효한 카메라 인덱스 목록: {available_indices}")
    return available_indices


def live_bbox_center_detector(camera_index: int, calc_roll: bool = True, min_area: int = 1000):
    """
    실시간 카메라 스트림에서 흰색 및 초록색 물체를 감지하고
    중앙 픽셀 좌표 (u, v) 및 (옵션) Roll 회전 각도를 계산하여 출력합니다.
    """
    cap = _open_capture(camera_index)

    if not cap.isOpened():
        print(f"FATAL ERROR: 카메라 인덱스 {camera_index}를 열 수 없습니다. 프로그램을 종료합니다.")
        sys.exit(1)

    # 색상 감지 범위 (HSV)
    # 1) 초록색
    lower_green = np.array([35, 50, 50], dtype=np.uint8)
    upper_green = np.array([85, 255, 255], dtype=np.uint8)
    # 2) 흰색 (낮은 채도, 높은 명도)
    lower_white = np.array([0, 0, 250], dtype=np.uint8)
    upper_white = np.array([180, 50, 255], dtype=np.uint8)

    COLOR_RED = (0, 0, 255)

    print(f"--- 🚀 실시간 좌표 감지 시작 (인덱스 {camera_index}, Q 키를 눌러 종료) ---")
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"🖼️ 현재 스트리밍 해상도: {actual_width} x {actual_height}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 받을 수 없습니다. 루프를 계속 재시도합니다.", end="\r")
            time.sleep(0.1)
            continue

        # BGR → HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 마스크 생성 및 합치기
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        final_mask = cv2.bitwise_or(mask_green, mask_white)

        # 컨투어 추출
        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > min_area:
                if calc_roll:
                    # 회전 바운딩 박스 (center, (w,h), angle)
                    rect = cv2.minAreaRect(largest_contour)
                    (u_c_float, v_c_float), (w, h), angle = rect
                    u_c, v_c = int(u_c_float), int(v_c_float)

                    # OpenCV angle 보정: 긴 변 기준으로 각도 표현 (선택적 규약)
                    roll_angle = angle + 90 if w < h else angle

                    # 시각화
                    box = cv2.boxPoints(rect)
                    box = box.astype(int)
                    cv2.drawContours(frame, [box], 0, COLOR_RED, 2)
                    cv2.circle(frame, (u_c, v_c), 5, COLOR_RED, -1)
                    text = f"P: ({u_c}, {v_c}) | Roll: {roll_angle:.2f}°"
                    cv2.putText(frame, text, (u_c - 120, v_c - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)

                    print(f"🎯 감지: u={u_c}, v={v_c}, Roll={roll_angle:.2f}° (Index: {camera_index})", end="\r")
                else:
                    # 단순 바운딩 박스
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    u_c, v_c = int(x + w / 2), int(y + h / 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), COLOR_RED, 2)
                    cv2.circle(frame, (u_c, v_c), 5, COLOR_RED, -1)
                    text = f"P: ({u_c}, {v_c})"
                    cv2.putText(frame, text, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)

                    print(f"🎯 좌표 감지: u={u_c}, v={v_c} (Index: {camera_index})", end="\r")
            else:
                print("상자 감지 안 됨 (면적 부족)", end="\r")
        else:
            print("상자 감지 안 됨 (컨투어 없음)", end="\r")

        cv2.imshow('Live Box Detection (Green/White) — Center & Roll', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="실시간 초록/흰색 객체 중심 및 (옵션) Roll 각도 검출기")
    parser.add_argument("--index", type=int, default=None, help="사용할 카메라 인덱스 (미지정 시 자동 탐색)")
    parser.add_argument("--no-roll", action="store_true", help="Roll 계산/표시 비활성화")
    parser.add_argument("--min-area", type=int, default=1000, help="검출 최소 면적(px^2)")
    parser.add_argument("--probe-max", type=int, default=3, help="자동 탐색 시 최대 시도 인덱스 (기본: 3)")
    args = parser.parse_args()

    # 카메라 인덱스 결정
    if args.index is not None:
        target_index = args.index
        print(f">> 사용할 카메라 인덱스(지정): {target_index}")
    else:
        available = find_camera_index(max_try=args.probe_max)
        if not available:
            print("\n유효한 카메라를 찾지 못했습니다. 프로그램을 종료합니다.")
            sys.exit(0)
        # 사용자가 3번째(인덱스 2)를 선호했던 힌트를 반영하되, 없으면 0 사용
        if len(available) > 2:
            target_index = available[2]
        else:
            target_index = available[0]
            print(f"\n⚠️ 주의: 2번 인덱스가 없어, {target_index}번 인덱스를 대신 사용합니다.")
        print(f">> 사용할 카메라 인덱스: {target_index}")

    live_bbox_center_detector(target_index, calc_roll=(not args.no_roll), min_area=args.min_area)


if __name__ == "__main__":
    main()
