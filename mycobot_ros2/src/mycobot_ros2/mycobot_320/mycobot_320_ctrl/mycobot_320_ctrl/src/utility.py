import numpy as np

# =========================
# 유틸
# =========================
def order_box_pts(pts4):
    """ cv2.boxPoints 결과를 TL, TR, BR, BL 순으로 정렬 """
    pts = np.array(pts4, dtype=np.float32)
    xs = np.argsort(pts[:,0])
    left, right = pts[xs[:2]], pts[xs[2:]]
    TL, BL = left[np.argsort(left[:,1])]
    TR, BR = right[np.argsort(right[:,1])]
    return TL, TR, BR, BL

def circ_mean_deg(deg_list):
    if not deg_list:
        return 0.0
    rads = np.radians(deg_list)
    x = np.cos(rads).mean()
    y = np.sin(rads).mean()
    return np.degrees(np.arctan2(y, x))

def norm180(a):
    return (a + 180.0) % 360.0 - 180.0

def get_pick_zone_rect(w: int, h: int, cx: int, cy: int, PICK_ZONE_REL_W, PICK_ZONE_REL_H):
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
