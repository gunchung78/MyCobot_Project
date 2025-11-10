import cv2
import numpy as np
from pymycobot.mycobot320 import MyCobot320
import time

mc = MyCobot320("COM7", 115200)

# =========================
# 색상 범위 (제공값 그대로 사용)
# =========================
COLOR_RANGES = {
    "red": [
        (np.array([0,   70,  70], dtype=np.uint8), np.array([10,  255, 255], dtype=np.uint8)),
        (np.array([170, 70,  70], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),  # 빨강 Hue 래핑
    ],
    "blue": [
        (np.array([90, 100, 80], dtype=np.uint8), np.array([110, 255, 255], dtype=np.uint8)),
    ],
    "green": [
        (np.array([35,  100, 125], dtype=np.uint8), np.array([85,  255, 255], dtype=np.uint8)),
    ],
}
# 시각화용 BGR 색상
COLOR_BRG_DRAW = {
    "red":    (0, 0, 255),
    "blue":   (255, 0, 0),
    "green":  (0, 255, 0),
}

# =========================
# 픽 존 설정 (기존 유지)
# =========================
PICK_ZONE_REL_W = 0.40
PICK_ZONE_REL_H = 0.70
PICK_CX = 420
PICK_CY = 220
PICK_ZONE_COLOR = (255, 0, 255)  # 분홍
PICK_ZONE_THICK = 2
FILTER_MASK_TO_ZONE = True

# 앵커(기준) 좌표
ANCHOR_PY = [222.8, -144.8, 248, 176.32, -0.69 , 0.06]

SX_TOP = 0.4     # X mm/px (상단)
SX_BOT = 0.4     # X mm/px (하단)
SCALE_X_MM_PER_PX = 0.4
SCALE_Y_MM_PER_PX = 0.4
Y_SIGN = -1

def get_pick_zone_rect(w: int, h: int, cx: int, cy: int):
    zw = max(6, int(w * PICK_ZONE_REL_W))
    zh = max(6, int(h * PICK_ZONE_REL_H))
    x1 = max(0, cx - zw // 2)
    y1 = max(0, cy - zh // 2)
    x2 = min(w - 1, x1 + zw)
    y2 = min(h - 1, y1 + zh)
    return (x1, y1, x2, y2)

def point_in_rect(x: int, y: int, rect) -> bool:
    x1, y1, x2, y2 = rect
    return (x1 <= x <= x2) and (y1 <= y <= y2)

def move_and_wait(mc, mode, value, speed=20, delay=1):
    if mode == "coords":
        mc.send_coords(value, speed, 0)
    elif mode == "angles":
        mc.send_angles(value, speed)
    else:
        raise ValueError("mode must be 'coords' or 'angles'")
    while mc.is_moving():
        time.sleep(0.1)
    time.sleep(delay)

cam = cv2.VideoCapture(0)
if not cam.isOpened():
    raise SystemExit("카메라를 열 수 없습니다.")

ok, frame0 = cam.read()
if not ok:
    raise SystemExit("첫 프레임을 읽지 못했습니다.")
H, W = frame0.shape[:2]
pick_zone = get_pick_zone_rect(W, H, PICK_CX, PICK_CY)

WAIT_SEC = 3.0
wait_start = None
returned_once = False

while True:
    ret, frame = cam.read()
    if not ret:
        print("프레임을 읽지 못했습니다.")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # =========================
    # ★ 다중 색상 마스크/컨투어 처리
    # =========================
    # 픽존 마스크 (필터링 옵션)
    zx1, zy1, zx2, zy2 = pick_zone
    zone_mask = None
    if FILTER_MASK_TO_ZONE:
        zone_mask = np.zeros((H, W), dtype=np.uint8)
        zone_mask[zy1:zy2+1, zx1:zx2+1] = 255

    # 색상별로 컨투어 검출/표시, 픽존 안 가장 큰 컨투어를 타깃으로
    output = frame.copy()
    cv2.rectangle(output, (zx1, zy1), (zx2, zy2), PICK_ZONE_COLOR, PICK_ZONE_THICK)
    cv2.putText(output, "PICK ZONE", (zx1, max(0, zy1-8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, PICK_ZONE_COLOR, 2)
    cv2.circle(output, (PICK_CX, PICK_CY), 6, (0, 0, 255), -1)

    center_uv = None
    target_color = None
    best_area = 0

    for cname, ranges in COLOR_RANGES.items():
        # 한 색상에 여러 구간(특히 red) → OR로 합침
        mask_c = np.zeros((H, W), dtype=np.uint8)
        for lo, hi in ranges:
            mask_c = cv2.bitwise_or(mask_c, cv2.inRange(hsv, lo, hi))
        if FILTER_MASK_TO_ZONE:
            mask_c = cv2.bitwise_and(mask_c, zone_mask)

        contours, _ = cv2.findContours(mask_c, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 시각화: 해당 색상의 모든 컨투어 표시
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:  # 너무 작은 노이즈 제거(원래 값보다 조금 낮게)
                continue
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect).astype(np.int32)
            (cx, cy), (w, h), angle = rect
            cx_i, cy_i = int(cx), int(cy)

            in_zone = point_in_rect(cx_i, cy_i, pick_zone)
            color_draw = COLOR_BRG_DRAW.get(cname, (200, 200, 200))
            cv2.drawContours(output, [box], 0, color_draw, 2)
            cv2.circle(output, (cx_i, cy_i), 5, color_draw, -1)
            cv2.putText(output, cname, (cx_i+6, cy_i-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_draw, 2)

            # 타깃 선택: 픽존 안 & 면적 최대
            if in_zone and area > 1000 and area > best_area:
                best_area = area
                center_uv = (cx_i, cy_i)
                target_color = cname

    # =========================
    # 이하 기존 로직 유지(좌표 계산/픽킹)
    # =========================
    if center_uv is not None:
        u, v = center_uv
        dx_pix = u - PICK_CX
        dy_pix = v - PICK_CY

        # 픽존 내부 y비율 (0=상단, 1=하단) → X 스케일 보간(기존 유지)
        y_frac = np.clip((v - zy1) / max(1, (zy2 - zy1)), 0.0, 1.0)
        scale_x_here = SX_TOP * (1.0 - y_frac) + SX_BOT * y_frac

        # ※ 아래 두 줄은 “기존 수식”을 그대로 둠
        dx_mm = -dx_pix * scale_x_here                      # 이미지 X → 로봇 X 보정(기존식)
        dy_mm = -dy_pix * SCALE_Y_MM_PER_PX * Y_SIGN        # 이미지 Y → 로봇 Y 보정(기존식)

        x_t = ANCHOR_PY[0] + dx_mm
        y_t = ANCHOR_PY[1] + dy_mm

        cv2.putText(output, f"Pix(u,v)=({u},{v}) [{target_color}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(output, f"Delta(px)=({dx_pix:+d},{dy_pix:+d})", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(output, f"Coord(x,y)=({x_t:.1f},{y_t:.1f})", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 2)

        if wait_start is None:
            wait_start = time.time()
        elapsed = time.time() - wait_start
        remaining = max(0.0, WAIT_SEC - elapsed)
        cv2.putText(output, f"waiting:{remaining:.1f}s",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        if (not returned_once) and elapsed >= WAIT_SEC:
            print(f"[RETURN] color={target_color}  x={x_t:.2f}, y={y_t:.2f}, pix=({u},{v}), area={best_area:.0f}")
            returned_once = True
            x = round(x_t, 2); y = round(y_t, 2)


            mc.power_on()
         
            move_and_wait(mc, "angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

            # 그리퍼 ON
            mc.set_gripper_mode(0)
            print("Gripper Mode", mc.get_gripper_mode())
            mc.init_electric_gripper()
            mc.set_electric_gripper(0)
            mc.set_gripper_value(80,20,1)

            move_and_wait(mc, "coords", [x, y, 240, 176.32, -0.69 , 0.06], 20)

            mc.set_gripper_value(10,20,1)
            time.sleep(2)

            move_and_wait(mc, "angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

            
            import box_moving_node as box
            box.place_box(mc, 'red', 2)

    else:
        wait_start = None

    if returned_once:
        cv2.putText(output, "READY (coords returned)", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)

    cv2.imshow("Frame", output)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    if key == ord('c'):
        wait_start = None
        returned_once = False
        print("[RESET] wait timer reset.")

cam.release()
cv2.destroyAllWindows()
