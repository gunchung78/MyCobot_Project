#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mycobot_320_vision.src.utility import *
from mycobot_320_vision.src.config_loader import load_config 

# ✅ config.json 로드 (전역)
C = load_config("config.json")

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.pub_result = self.create_publisher(Float32MultiArray, '/detector_result', 10)

        # ===============================
        # 설정값 초기화
        # ===============================
        self.angles_buf = deque(maxlen=C.ANGLE_WINDOW)
        self.wait_start = None

        # ✅ 카메라 초기화
        self.cap = cv2.VideoCapture(C.CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ 카메라를 열 수 없습니다. (CAM_INDEX={C.CAM_INDEX})")
            raise SystemExit
        ok, frame0 = self.cap.read()
        if not ok:
            raise SystemExit("첫 프레임을 읽지 못했습니다.")
        self.H, self.W = frame0.shape[:2]

        # ✅ 픽존(작업 영역) 설정
        self.pick_zone = get_pick_zone_rect(
            self.W, self.H, C.PICK_CX, C.PICK_CY,
            C.PICK_ZONE_REL_W, C.PICK_ZONE_REL_H
        )
        self.zx1, self.zy1, self.zx2, self.zy2 = self.pick_zone

        # ✅ 루프 타이머
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("✅ DetectorNode initialized and running.")

    # ===============================
    # Main Loop
    # ===============================
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()

        # ✅ 픽존 시각화
        cv2.rectangle(overlay, (self.zx1, self.zy1), (self.zx2, self.zy2),
                      tuple(C.PICK_ZONE_COLOR), C.PICK_ZONE_THICK)
        cv2.putText(overlay, "PICK ZONE", (self.zx1, max(0, self.zy1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        selected = None

        # ✅ 색상별 탐지
        for color_name, ranges in C.COLOR_RANGES.items():
            color_mask = np.zeros((self.H, self.W), dtype=np.uint8)
            for (lo, hi) in ranges:
                color_mask |= cv2.inRange(hsv, lo, hi)

            # 형태학적 연산 (옵션)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (C.MORPH_KERNEL_SIZE, C.MORPH_KERNEL_SIZE))
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=C.MORPH_OPEN_ITER)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel, iterations=C.MORPH_CLOSE_ITER)

            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area < C.MIN_AREA:
                continue

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect).astype(np.int32)
            cx, cy = int(rect[0][0]), int(rect[0][1])

            if not point_in_rect(cx, cy, self.pick_zone):
                continue

            # 회전 각도 계산
            TL, TR, BR, BL = order_box_pts(box)
            midL = ((TL + BL) * 0.5).astype(int)
            midR = ((TR + BR) * 0.5).astype(int)
            vx = float(midR[0] - midL[0])
            vy = float(midR[1] - midL[1])
            angle_here = math.degrees(math.atan2(vy, vx))

            dx = cx - C.PICK_CX
            dy = cy - C.PICK_CY
            dist2 = dx * dx + dy * dy

            candidate = {
                "color": color_name,
                "box": box,
                "center": (cx, cy),
                "angle_img": angle_here,
                "dist2": dist2
            }
            if selected is None or candidate["dist2"] < selected["dist2"]:
                selected = candidate

        # ✅ 인식 실패 시
        if selected is None:
            cv2.imshow("Detector", overlay)
            cv2.waitKey(1)
            return

        # ✅ 인식 성공 시 박스 시각화
        box = selected["box"]
        color_name = selected["color"]

        draw_color = tuple(C.COLOR_BRG_DRAW[color_name])

        cv2.drawContours(overlay, [box], 0, draw_color, 2)
        cx, cy = selected["center"]
        cv2.circle(overlay, (cx, cy), 6, draw_color, -1)

        TL, TR, BR, BL = order_box_pts(box)
        midL = ((TL + BL) * 0.5).astype(int)
        midR = ((TR + BR) * 0.5).astype(int)
        cv2.line(overlay, tuple(midL), tuple(midR), draw_color, 2)
        cv2.putText(overlay, color_name.upper(), (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

        dx_pix = cx - C.PICK_CX
        dy_pix = cy - C.PICK_CY
        dx_mm = dx_pix * C.SCALE_X_MM_PER_PX
        dy_mm = dy_pix * C.SCALE_Y_MM_PER_PX
        x_t = C.ANCHOR_PY[0] - dx_mm
        y_t = C.ANCHOR_PY[1] + dy_mm + C.CAMERA_MM

        angle_here = selected["angle_img"]
        self.angles_buf.append(angle_here)
        rz_t = None
        if len(self.angles_buf) >= 3:
            mean_angle = circ_mean_deg(list(self.angles_buf))
            rz_t = norm180(-mean_angle + C.CAL_RZ_OFFSET)

        color_name = selected["color"]

        # ✅ 퍼블리시 데이터 구성
        color_map = {"red": 1.0, "blue": 2.0, "green": 3.0}
        msg = Float32MultiArray()
        msg.data = [
            float(x_t),
            float(y_t),
            float(rz_t if rz_t else 0.0),
            color_map.get(color_name, 0.0)
        ]
        self.pub_result.publish(msg)

        # ✅ 디버그 텍스트 및 시각화 표시
        cv2.rectangle(overlay, (self.zx1, self.zy1), (self.zx2, self.zy2), C.PICK_ZONE_COLOR, C.PICK_ZONE_THICK)
        cv2.circle(overlay, (C.PICK_CX, C.PICK_CY), 6, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f"x={x_t:.1f}, y={y_t:.1f}, rz={(rz_t if rz_t is not None else 0.0):.1f}, color={color_name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )
        cv2.drawContours(overlay, [box], 0, draw_color, 2)
        cv2.imshow("Detector", overlay)
        cv2.waitKey(1)

    # ===============================
    # 종료 처리
    # ===============================
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
