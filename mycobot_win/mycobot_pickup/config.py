# -*- coding: utf-8 -*-
import numpy as np

# ── 연결 ─────────────────────────────────────────
PORT = "COM3"       # Win 예시: "COM3" / Linux 예시: "/dev/ttyUSB0"
BAUD = 115200

# ── 카메라 ───────────────────────────────────────
CAM_INDEX = 1

# ── 픽존 ────────────────────────────────────────
PICK_ZONE_REL_W = 0.40
PICK_ZONE_REL_H = 0.50
PICK_CX = 320
PICK_CY = 240
PICK_ZONE_COLOR = (255, 0, 255)
PICK_ZONE_THICK = 2
FILTER_MASK_TO_ZONE = True

# ── 앵커/좌표 보정 ──────────────────────────────
ANCHOR_PY = [192.0, 0.0, 300.0, -180.0, 0.0, 0.0]
SCALE_X_MM_PER_PX = 0.42
SCALE_Y_MM_PER_PX = 0.42
CAMERA_MM = -85  # 카메라/작업대 오프셋을 Y축에 더함(+/- 환경에 맞게)

# ── RZ 보정/적용 ───────────────────────────────
CAL_RZ_OFFSET = 0.0
APPLY_RZ = True  # True로 바꾸면 회전 픽킹 적용

# ── Vision.py ──────────────────────────────────
MODE = 0 # mode = 0 정상 이상 분별, mode = 1 물체 적제
VISION_RESULT = [0, 1]

# 탐지 파라미터
MIN_AREA = 1000
MORPH_KERNEL_SIZE = 3
MORPH_OPEN_ITER   = 1
MORPH_CLOSE_ITER  = 1

# ========= 색상별 HSV 범위 =========
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

# 시각화용 BGR 색상 맵(컨투어/박스 라벨)
COLOR_BRG_DRAW = {
    "red":    (0, 0, 255),
    "blue":   (255, 0, 0),
    "green":  (0, 255, 0),
}


# ── 타이밍 ──────────────────────────────────────
WAIT_SEC = 3.0
ANGLE_WINDOW = 15

# ── 이동/그리퍼 ─────────────────────────────────
MOVE_SPEED = 40
APPROACH_Z = 248
GRIPPER_OPEN_VAL = 100
GRIPPER_CLOSE_VAL = 10
GRIPPER_SPEED = 20
