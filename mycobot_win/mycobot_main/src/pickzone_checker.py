# pickzone_checker.py

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
import cv2, numpy as np, time

# ==== 픽존 & 색상 상수 ====
PICK_ZONE_REL_W = 0.55
PICK_ZONE_REL_H = 0.50
PICK_CX = 400
PICK_CY = 200
PICK_ZONE_COLOR = (255, 0, 255)
PICK_ZONE_THICK = 2
FILTER_MASK_TO_ZONE = True
VIEW_ROT_DEG = -0.0

COLOR_RANGES = {
    "red":  [(np.array([0,70,70],np.uint8),  np.array([10,255,255],np.uint8)),
             (np.array([170,70,70],np.uint8),np.array([179,255,255],np.uint8))],
    "blue": [(np.array([90,60,80],np.uint8), np.array([110,255,255],np.uint8))],
    "green":[(np.array([35,90,100],np.uint8), np.array([85,255,255],np.uint8))],
    "white":[(np.array([0,0,180],np.uint8),   np.array([180,30,255],np.uint8))],
}
COLOR_BGR = {"red":(0,0,255),"blue":(255,0,0),"green":(0,255,0),"white":(0,0,0)}

# --- dual_cam_probe 스타일: 한 함수로 카메라 열기 ---
def open_cam(src):
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)  # Ubuntu 22.04 권장
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # 필요하면 해상도 지정:
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap

def _get_rotated_pick_zone(w,h,cx,cy,angle_deg):
    zw = max(6, int(w*PICK_ZONE_REL_W)); zh = max(6, int(h*PICK_ZONE_REL_H))
    rrect = ((float(cx),float(cy)), (float(zw),float(zh)), float(angle_deg))
    box = cv2.boxPoints(rrect).astype(np.int32)
    return rrect, box, zw*zh

def _point_in_rotated_rect(x,y,poly4):
    return cv2.pointPolygonTest(poly4, (float(x),float(y)), False) >= 0


class PickZoneChecker:
    """비블로킹 초기화 + 한프레임/짧은투표 확인용 (dual_cam_probe 방식)"""
    def __init__(self, cam_index="/dev/video2", cap=None, discard=4):
        self.src = cam_index   # "/dev/video2" 또는 2
        self.cap = cap
        self.discard = discard
        self._H = self._W = None
        self._zone_pts = None
        self._zone_mask = None
        self._area_th = 600
        self._kernel = np.ones((5,5), np.uint8)

    def start(self):
        if self.cap is None:
            self.cap = open_cam(self.src)
        if not self.cap.isOpened():
            raise SystemExit(f"카메라를 열 수 없습니다: {self.src}")

        # 최신 프레임 확보
        for _ in range(self.discard):
            self.cap.grab()
        ok, f0 = self.cap.retrieve()
        if not ok or f0 is None:
            self.cap.release(); self.cap = None
            raise SystemExit("첫 프레임을 읽지 못했습니다.")

        self._H, self._W = f0.shape[:2]
        _, self._zone_pts, area = _get_rotated_pick_zone(self._W, self._H, PICK_CX, PICK_CY, VIEW_ROT_DEG)
        self._zone_mask = np.zeros((self._H, self._W), np.uint8)
        cv2.fillPoly(self._zone_mask, [self._zone_pts], 255)
        self._area_th = max(600, int(area*0.002))
        print(f"[PickZoneChecker] ready cam={self.src} size={self._W}x{self._H}")

    def stop(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _read_latest(self):
        for _ in range(self.discard):
            self.cap.grab()
        return self.cap.retrieve()

    # pickzone_checker.py (발췌)
    def detect_once(self, show: bool = False):
        ok, frame = self._read_latest()
        if not ok or frame is None:
            return {"ok": False, "has_target": False, "color": None,
                    "center": None, "angle": None, "overlay": None}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()
        cv2.polylines(overlay, [self._zone_pts], True, PICK_ZONE_COLOR, PICK_ZONE_THICK)
        cv2.circle(overlay, (PICK_CX, PICK_CY), 6, (0,0,255), -1)

        found=False; f_color=None; f_center=None; f_angle=None
        for cname, ranges in COLOR_RANGES.items():
            mask = np.zeros((self._H, self._W), np.uint8)
            for lo,hi in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            mask = cv2.bitwise_and(mask, self._zone_mask)

            contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            col = COLOR_BGR.get(cname,(200,200,200))
            for cnt in contours:
                if cv2.contourArea(cnt) <= self._area_th: continue
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect).astype(np.int32)
                (cx,cy),(w,h),angle = rect
                cx_i,cy_i = int(cx),int(cy)
                in_zone = _point_in_rotated_rect(cx_i,cy_i,self._zone_pts)
                cv2.drawContours(overlay,[box],0, col if in_zone else (160,160,160),2)
                cv2.circle(overlay,(cx_i,cy_i),5, col if in_zone else (160,160,160),-1)
                if in_zone and not found:
                    found=True; f_color=cname; f_center=(cx_i,cy_i); f_angle=angle

        return {"ok": True, "has_target": found, "color": f_color,
                "center": f_center, "angle": f_angle, "overlay": overlay}
                
    # 2) check_presence: show 인자를 받아도 '무시'
    def check_presence(self, timeout_sec: float = 0.8, require_consecutive: int = 2, show: bool = False):
        hits = 0
        last = None
        end = time.time() + timeout_sec
        while time.time() < end:
            last = self.detect_once()  # ← 항상 False로 호출
            if not last.get("ok"):
                continue
            if last.get("has_target"):
                hits += 1
                if hits >= require_consecutive:
                    return True, last
            else:
                hits = 0
        return False, last

    def preview_loop(self):
        while True:
            info = self.detect_once(show=True)
            if not info["ok"]: break
            if cv2.waitKey(1)&0xFF==27: break
