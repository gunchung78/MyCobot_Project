# -*- coding: utf-8 -*-
from typing import List, Sequence, Optional


class ROS_Robot:
    def __init__(self, mode, C):
        self.move_speed = int(C.MOVE_SPEED)
        self.gr_open = int(C.GRIPPER_OPEN_VAL)
        self.gr_close = int(C.GRIPPER_CLOSE_VAL)
        self.gr_speed = int(C.GRIPPER_SPEED)
        self.ANCHOR_PY = list(C.ANCHOR_PY)
        self.mode = mode
        self.result = []
        self.placeList= [0,0,0]

    def move_and_wait(self, mode: str, value: Sequence[float], speed: Optional[int]=None,
                    delay: float=0.5, poll_sec: float=0.1):
        """
        비동기 명령 후 is_moving()으로 정지까지 대기.
        - mode = 'coords' : send_coords(value, speed, mode=0)
        - mode = 'angles' : send_angles(value, speed)
        """
        try:
            spd = speed or self.move_speed
            # if mode == "coords":
            #     self.mc.send_coords(list(value), spd, 0)
            # elif mode == "angles":
            #     self.mc.send_angles(list(value), spd)
            if mode in ("coords", "angles"):
                self.result.append([mode, list(value)])
            else:
                raise ValueError("mode must be 'coords' or 'angles'")
        except Exception as e:
            print(f"[WARN] move_and_wait 실패: {e}")

    # ── 전원/기본 이동 ─────────────────────────────
    def power_on(self, go_anchor: bool=True):
        try:
            if go_anchor:
                self.go_anchor()
        except Exception as e:
            print(f"[WARN] power_on 실패: {e}")
 
    # ── 그리퍼 호환 래퍼 ──────────────────────────
    def gripper_open(self):
        try:
            # self.mc.set_gripper_value(self.gr_open, self.gr_speed, 1)
            self.result.append(["gripper", True])
        except Exception as e:
            print(f"[WARN] gripper open 실패: {e}")

    def gripper_close(self):
        try:
            # self.mc.set_gripper_value(self.gr_close, self.gr_speed, 1)
            self.result.append(["gripper", False])
        except Exception as e:
            print(f"[WARN] gripper close 실패: {e}")

    # ── 분류/적재 동작(색상별) ─────────────────────
    def place_box(self, val: str, idx: int=0):
        """
        분류 색상(val)에 따라 지정된 드롭존으로 이동 후 놓기.
        - val: 'green'/'normal', 'blue', 'red'/'anomaly'
        - idx: 적재 층(0,1,2,...) → z 오프셋
        """
        z = idx * 25
        try:
            if val in ('green', 'normal'):
                z = z + 148
                # 접근 자세(관절)
                self.move_and_wait("angles", [-10, 0, 78.95, -21, -87.36, -15])
                # 드롭 좌표(툴 좌표)
                self.move_and_wait("coords", [-293.5, -25, z, -176, 0, 90], speed=10, delay=1.0)
                # 놓기
                # self.mc.set_gripper_value(40, 20, 1)
                self.gripper_open()
                # 이탈 자세
                self.move_and_wait("angles", [-22.23, 0, 78.95, -21, -87.36, -15]) 

            elif val == 'blue':
                z = z + 170
                self.move_and_wait("angles", [12.12, 0, 70.83, -16.08, -67.5, -150])
                self.move_and_wait("coords", [-219.2, -281.9, z, 179.41, -5, -100], speed=10, delay=1.0)
                # self.mc.set_gripper_value(40, 20, 1)
                self.gripper_open()
                self.move_and_wait("angles", [12.12, 0, 70.83, -16.08, -67.5, -150])

            elif val == 'red':
                z = z + 160
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])
                self.move_and_wait("coords", [-180, 240, z, -173.15, 0, 90], speed=10, delay=1.0)
                # self.mc.set_gripper_value(40, 20, 1)
                self.gripper_open()
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])

            elif val == 'anomaly':
                self.move_and_wait("angles", [-80, -20, -65, -0, 90, -90])
                # self.mc.set_gripper_value(40, 20, 1)
                self.gripper_open()
              
            else:
                print(f"[WARN] place_box: 알 수 없는 색상 '{val}' (동작 생략)")

            # # 공통 복귀 시퀀스
            self.go_anchor()

        except Exception as e:
            print(f"[ERROR] place_box 실패: {e}")

    # ── 편의 함수 ─────────────────────────────────
    def go_anchor(self, speed=20, delay=0.2):
        try:
            # 웨이포인트 먼저 이동
            self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25], speed, delay)
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] go_anchor 실패: {e}")

    def refresh_home(self, speed=20, delay=0.2):
        try:
            self.move_and_wait("angles", [0, 10, -80, -0, 90, -90], speed, delay)
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] go_anchor 실패: {e}")

    def main_fow(self, x_t, y_t, rz_t, color, detected_type):
        self.result.clear()
        if rz_t is None:
            rz_t = self.ANCHOR_PY[5]
        try:
            if self.mode == "detect_only" and (color == "white" or color is None ):
                print(f'no color {color}')
                self.refresh_home()
                return list(self.result)
            
            elif self.mode == "detect_and_classify" and (detected_type ==  "unknown" or detected_type is None): 
                print(f'no detected_type {detected_type}')
                self.refresh_home()
                return list(self.result)
            
            else:
                # 접근(상부)
                # r.move_coords([x_t, y_t, C.ANCHOR_PY[2]+30, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

                # r.move_coords([x_t, y_t, C.APPROACH_Z, C.ANCHOR_PY[3], C.ANCHOR_PY[4], rz_t], C.MOVE_SPEED, 0, sleep=0.5)
                self.move_and_wait(
                    "coords",
                    [x_t, y_t, self.ANCHOR_PY[2], self.ANCHOR_PY[3], self.ANCHOR_PY[4], rz_t],
                    self.move_speed,
                    0
                )

                # 집기
                self.gripper_close()
                # 복귀
                self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

                # [변경된 분류 로직]
                # 색상 + YOLO 결과 모두 고려
                if self.mode == "detect_only":
                    if color == "red":
                        print("[ACTION] red 감지")
                        self.place_box("red", self.placeList[0])
                        self.placeList[0] += 1
                    elif color == "green":
                        print("[ACTION] green 감지")
                        self.place_box("green", self.placeList[2])
                        self.placeList[2] += 1
                    elif color == "blue":
                        print("[ACTION] blue 감지")
                        self.place_box("blue", self.placeList[1])
                        self.placeList[1] += 1
                elif self.mode == "detect_and_classify": 
                    # 추가로 YOLO 결과에 따라 색상 불분명시 대체 동작 수행
                    if detected_type == "anomaly":
                        print("[ACTION] YOLO anomaly 판정")
                        self.place_box("anomaly")
                    elif detected_type == "normal":
                        print("[ACTION] YOLO normal 판정")
                        self.place_box("normal", self.placeList[2])
                        self.placeList[2] += 1
                    else:
                        print("[WARN] 색상 및 YOLO 판정 불명 → 동작 생략")
            return list(self.result)
        except Exception as e:
            print(f"[ERROR] 로봇 명령 실패: {e}")

# ===== 여기부터 테스트 실행용 main (기존 코드 변경 없음) =====
if __name__ == "__main__":
    from config_loader import load_config 
    C = load_config("../config.json")

    # 샘플 입력값
    mode = "detect_only"                 # 1: color 기반, 0: yolo 기반
    x_t, y_t, rz_t = -230.0, -50.0, 0.0
    color = "green"
    detected_type = "normal"

    # 인스턴스 생성
    bot = ROS_Robot(mode, C)
    print("\n=== Test Result * 3 ===")
    for i in range(3):
    # 메인 플로우 실행
        out = bot.main_fow(x_t, y_t, rz_t, color, detected_type)
        # 결과 출력
        print(i)
        for step in out:
            print(step)
