# detector.py
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import time
from collections import deque

class PickTargetDetector:
    """
    프레임에서 [빨/파/초] 물체를 탐지하고,
    - 중앙과 가장 가까운 1개 후보 선택
    - 픽셀 오프셋 → mm 보정
    - 앵커 기준 타겟 좌표 (x_t, y_t)
    - 회전 각도 안정화 평균 → rz_t
    를 계산해 반환한다.

    반환(dict):
      has_target: bool
      x_t, y_t: float (mm)
      rz_t: float (deg)
      robot_action: bool   # True면 픽킹 실행 조건 충족
      color: str | None
      u, v: int            # 선택 박스 중심 픽셀
      dx_pix, dy_pix: int  # 카메라 중앙 대비 픽셀 오프셋
      angle_img_raw: float | None
      angle_img_stable: float | None
      overlay: np.ndarray  # 시각화용 프레임
      debug: dict          # 추가 디버그 정보
    """

    def __init__(self, C, util):
        self.C = C
        self.util = util

        # 내부 상태(평균용)
        self.angles_buf = deque(maxlen=C.ANGLE_WINDOW)
        self.wait_start = None

        # 픽존 캐시 (W,H 알아야 하므로 set_frame_size 이후 채움)
        self.zx1 = self.zy1 = self.zx2 = self.zy2 = None
        self.pick_zone = None
        self.frame_W = self.frame_H = None

    # ---- 외부에서 최초 1회: 카메라 해상도 알려주기 ----
    def set_frame_size(self, W, H):
        self.frame_W, self.frame_H = W, H
        self.pick_zone = self.util.get_pick_zone_rect(
            W, H, self.C.PICK_CX, self.C.PICK_CY,
            self.C.PICK_ZONE_REL_W, self.C.PICK_ZONE_REL_H
        )
        self.zx1, self.zy1, self.zx2, self.zy2 = self.pick_zone

    # ---- 상태 초기화 ----
    def reset(self):
        self.angles_buf.clear()
        self.wait_start = None

    # ---- 핵심: 1프레임 처리 ----
    def process(self, frame):
        assert self.frame_W is not None, "set_frame_size() 먼저 호출하세요."
        C, util = self.C, self.util
        H, W = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()

        # 픽존 마스크
        zone_mask = None
        if C.FILTER_MASK_TO_ZONE:
            zone_mask = np.zeros((H, W), dtype=np.uint8)
            zone_mask[self.zy1:self.zy2+1, self.zx1:self.zx2+1] = 255

        # 중앙과 제일 가까운 후보 1개 선택
        selected = None  # dict: {color,cnt,box,center,angle_img,area,dist2,midL,midR}
        for color_name, ranges in C.COLOR_RANGES.items():
            color_mask = np.zeros((H, W), dtype=np.uint8)
            for (lo, hi) in ranges:
                color_mask |= cv2.inRange(hsv, lo, hi)

            if C.FILTER_MASK_TO_ZONE and zone_mask is not None:
                color_mask = cv2.bitwise_and(color_mask, zone_mask)

            # 노이즈 제거
            kernel = np.ones((C.MORPH_KERNEL_SIZE, C.MORPH_KERNEL_SIZE), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=C.MORPH_OPEN_ITER)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel, iterations=C.MORPH_CLOSE_ITER)

            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area <= C.MIN_AREA:
                continue

            rect = cv2.minAreaRect(cnt)  # ((cx,cy),(w,h),ang)
            box = cv2.boxPoints(rect).astype(np.int32)
            cx, cy = int(rect[0][0]), int(rect[0][1])

            # 픽존 내부만 유효
            if not util.point_in_rect(cx, cy, self.pick_zone):
                continue

            TL, TR, BR, BL = util.order_box_pts(box)
            midL = ((TL + BL) * 0.5).astype(int)
            midR = ((TR + BR) * 0.5).astype(int)
            vx = float(midR[0] - midL[0])
            vy = float(midR[1] - midL[1])
            angle_here = math.degrees(math.atan2(vy, vx))  # 이미지 좌표계 기준

            dx = cx - C.PICK_CX
            dy = cy - C.PICK_CY
            dist2 = dx*dx + dy*dy

            candidate = {
                "color": color_name, "cnt": cnt, "box": box,
                "center": (cx, cy), "angle_img": angle_here,
                "area": area, "dist2": dist2, "midL": midL, "midR": midR
            }
            if (selected is None) or (candidate["dist2"] < selected["dist2"]):
                selected = candidate

        # 픽존 시각화
        cv2.rectangle(overlay, (self.zx1, self.zy1), (self.zx2, self.zy2), C.PICK_ZONE_COLOR, C.PICK_ZONE_THICK)
        cv2.putText(overlay, "PICK ZONE", (self.zx1, max(0, self.zy1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C.PICK_ZONE_COLOR, 2)
        cv2.circle(overlay, (C.PICK_CX, C.PICK_CY), 6, (0, 0, 255), -1)

        # 기본 반환값
        ret = {
            "has_target": False,
            "x_t": None, "y_t": None,
            "rz_t": None,
            "robot_action": False,
            "color": None,
            "u": None, "v": None,
            "dx_pix": None, "dy_pix": None,
            "angle_img_raw": None,
            "angle_img_stable": None,
            "overlay": overlay,
            "debug": {}
        }

        if selected is None:
            # 타겟 없음: 안정화 타이머/버퍼 리셋
            self.reset()
            return ret

        # --- 시각화 ---
        color_name = selected["color"]
        draw_col = getattr(C.COLOR_BRG_DRAW, color_name, (255, 255, 255))
        cv2.drawContours(overlay, [selected["box"]], 0, draw_col, 2)
        cv2.circle(overlay, tuple(selected["midL"]), 6, draw_col, -1)
        cv2.circle(overlay, tuple(selected["midR"]), 6, draw_col, -1)
        cv2.line(overlay, tuple(selected["midL"]), tuple(selected["midR"]), draw_col, 2)
        cv2.putText(overlay, f"{color_name.upper()}",
                    (selected["center"][0]+8, selected["center"][1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_col, 2)

        # --- 좌표/각도 계산 ---
        u, v = selected["center"]
        dx_pix = int(u - C.PICK_CX)
        dy_pix = int(v - C.PICK_CY)

        dx_mm = dx_pix * C.SCALE_X_MM_PER_PX
        dy_mm = dy_pix * C.SCALE_Y_MM_PER_PX

        x_t = C.ANCHOR_PY[0] - dx_mm + C.CAMERAX_MM
        y_t = C.ANCHOR_PY[1] + dy_mm + C.CAMERAY_MM


        angle_img_raw = float(selected["angle_img"])
        if angle_img_raw is not None:
            self.angles_buf.append(angle_img_raw)

        # 타이머 시작
        if self.wait_start is None:
            self.wait_start = time.time()

        elapsed = time.time() - self.wait_start
        remaining = max(0.0, C.WAIT_SEC - elapsed)
        cv2.putText(overlay, f"waiting:{remaining:.1f}s",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        angle_img_stable = None
        rz_t = None
        robot_action = False

        if len(self.angles_buf) >= max(5, C.ANGLE_WINDOW // 2) and elapsed >= C.WAIT_SEC:
            angle_img_stable = self.util.circ_mean_deg(list(self.angles_buf))
            rz_deg = self.util.norm180(-angle_img_stable + C.CAL_RZ_OFFSET)
            rz_t = C.ANCHOR_PY[5] if not C.APPLY_RZ else rz_deg
            robot_action = True

        # HUD
        cv2.putText(overlay, f"Pix(u,v)=({u},{v})", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(overlay, f"Delta(px)=({dx_pix:+d},{dy_pix:+d})", (10,55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(overlay, f"Coord(x,y)=({x_t:.1f},{y_t:.1f})", (10,80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 2)

        # 반환 채우기
        ret.update({
            "has_target": True,
            "x_t": float(x_t), "y_t": float(y_t),
            "rz_t": None if rz_t is None else float(rz_t),
            "robot_action": bool(robot_action),
            "color": color_name,
            "u": int(u), "v": int(v),
            "dx_pix": dx_pix, "dy_pix": dy_pix,
            "angle_img_raw": None if angle_img_raw is None else float(angle_img_raw),
            "angle_img_stable": None if angle_img_stable is None else float(angle_img_stable),
            "overlay": overlay,
            "debug": {
                "elapsed": float(elapsed),
                "remaining": float(remaining),
                "angles_buf_len": len(self.angles_buf)
            }
        })
        return ret
