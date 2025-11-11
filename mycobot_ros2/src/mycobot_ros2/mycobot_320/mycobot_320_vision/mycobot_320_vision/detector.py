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
from mycobot_320_vision.utility import *


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.pub_result = self.create_publisher(Float32MultiArray, '/detector_result', 10)

        # ===============================
        # Config (config.json 내용 반영)
        # ===============================
        self.PORT = "/dev/ttyACM0"
        self.BAUD = 115200
        self.CAM_INDEX = 0
        self.MODEL_PATH_DIR = "mycobot_320_vision/mycobot_320_vision/data/best.pt"
        self.YAML_PATH_DIR = "mycobot_320_vision/mycobot_320_vision/data/custom_data.yaml"

        self.PICK_CX = 320
        self.PICK_CY = 240
        self.PICK_ZONE_REL_W = 0.4
        self.PICK_ZONE_REL_H = 0.5
        self.PICK_ZONE_COLOR = (0, 255, 255)  # (B,G,R)
        self.PICK_ZONE_THICK = 2
        self.FILTER_MASK_TO_ZONE = True

        self.ANCHOR_PY = [192.0, 0.0, 300.0, -180.0, 0.0, 0.0]

        self.SCALE_X_MM_PER_PX = 0.35
        self.SCALE_Y_MM_PER_PX = 0.42
        self.CAMERA_MM = -85
        self.CAL_RZ_OFFSET = 0.0
        self.APPLY_RZ = True

        self.MODE = 0
        self.VISION_RESULT = [0, 1]
        self.MIN_AREA = 1000
        self.MORPH_KERNEL_SIZE = 3
        self.MORPH_OPEN_ITER = 1
        self.MORPH_CLOSE_ITER = 1

        self.COLOR_RANGES = {
            "red": [
                ((0, 70, 70), (10, 255, 255)),
                ((170, 70, 70), (179, 255, 255))
            ],
            "blue": [
                ((90, 100, 80), (110, 255, 255))
            ],
            "green": [
                ((35, 100, 125), (85, 255, 255))
            ]
        }

        self.COLOR_BRG_DRAW = {
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
            "green": (0, 255, 0)
        }

        self.WAIT_SEC = 3.0
        self.ANGLE_WINDOW = 15
        self.MOVE_SPEED = 30
        self.APPROACH_Z = 243
        self.GRIPPER_OPEN_VAL = 150
        self.GRIPPER_CLOSE_VAL = 10
        self.GRIPPER_SPEED = 20
        # ===============================

        # 상태
        self.angles_buf = deque(maxlen=self.ANGLE_WINDOW)
        self.wait_start = None

        # 카메라 초기화
        self.cap = cv2.VideoCapture(self.CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라를 열 수 없습니다.")
            raise SystemExit
        ok, frame0 = self.cap.read()
        if not ok:
            raise SystemExit("첫 프레임을 읽지 못했습니다.")
        self.H, self.W = frame0.shape[:2]

        # 영역 설정
        self.pick_zone = get_pick_zone_rect(
            self.W, self.H, self.PICK_CX, self.PICK_CY,
            self.PICK_ZONE_REL_W, self.PICK_ZONE_REL_H
        )
        self.zx1, self.zy1, self.zx2, self.zy2 = self.pick_zone

        # 루프 타이머
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("✅ DetectorNode initialized and running.")

    # ===============================
    # Main loop
    # ===============================
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()

        # 픽존 표시
        cv2.rectangle(overlay, (self.zx1, self.zy1), (self.zx2, self.zy2),
                      self.PICK_ZONE_COLOR, self.PICK_ZONE_THICK)
        cv2.putText(overlay, "PICK ZONE", (self.zx1, max(0, self.zy1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        selected = None

        for color_name, ranges in self.COLOR_RANGES.items():
            color_mask = np.zeros((self.H, self.W), dtype=np.uint8)
            for (lo, hi) in ranges:
                color_mask |= cv2.inRange(hsv, lo, hi)

            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area < self.MIN_AREA:
                continue

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect).astype(np.int32)
            cx, cy = int(rect[0][0]), int(rect[0][1])

            if not point_in_rect(cx, cy, self.pick_zone):
                continue

            TL, TR, BR, BL = order_box_pts(box)
            midL = ((TL + BL) * 0.5).astype(int)
            midR = ((TR + BR) * 0.5).astype(int)
            vx = float(midR[0] - midL[0])
            vy = float(midR[1] - midL[1])
            angle_here = math.degrees(math.atan2(vy, vx))

            dx = cx - self.PICK_CX
            dy = cy - self.PICK_CY
            dist2 = dx*dx + dy*dy

            candidate = {
                "color": color_name,
                "box": box,
                "center": (cx, cy),
                "angle_img": angle_here,
                "dist2": dist2
            }
            if (selected is None) or (candidate["dist2"] < selected["dist2"]):
                selected = candidate

        if selected is None:
            cv2.imshow("Detector", overlay)
            cv2.waitKey(1)
            return

        # 좌표 계산
        cx, cy = selected["center"]
        dx_pix = cx - self.PICK_CX
        dy_pix = cy - self.PICK_CY
        dx_mm = dx_pix * self.SCALE_X_MM_PER_PX
        dy_mm = dy_pix * self.SCALE_Y_MM_PER_PX
        x_t = self.ANCHOR_PY[0] - dx_mm
        y_t = self.ANCHOR_PY[1] + dy_mm + self.CAMERA_MM

        angle_here = selected["angle_img"]
        self.angles_buf.append(angle_here)
        rz_t = None
        if len(self.angles_buf) >= 3:
            mean_angle = circ_mean_deg(list(self.angles_buf))
            rz_t = norm180(-mean_angle + self.CAL_RZ_OFFSET)

        color_name = selected["color"]

        # publish
        color_map = {"red": 1.0, "blue": 2.0, "green": 3.0}
        msg = Float32MultiArray()
        msg.data = [
            float(x_t),
            float(y_t),
            float(rz_t if rz_t else 0.0),
            color_map.get(color_name, 0.0)
        ]
        self.pub_result.publish(msg)

        cv2.putText(
            overlay,
            f"x={x_t:.1f}, y={y_t:.1f}, rz={(rz_t if rz_t is not None else 0.0):.1f}, color={color_name}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        cv2.imshow("Detector", overlay)
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
