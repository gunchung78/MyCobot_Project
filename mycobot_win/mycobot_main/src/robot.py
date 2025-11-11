# -*- coding: utf-8 -*-
import time
from typing import List, Sequence, Optional
from pymycobot.mycobot320 import MyCobot320

class Robot:
    """
    MyCobot320 제어 래퍼.
    - 전원/기본 이동/그리퍼
    - 분류 적재(place_box) 동작 내장
    """

    def __init__(self,
                 port: str,
                 baud: int,
                 move_speed: int,
                 gripper_open_val: int,
                 gripper_close_val: int,
                 gripper_speed: int,
                 ANCHOR_PY: Sequence[float]):
        self.mc = MyCobot320(port, baud)
        self.move_speed = int(move_speed)
        self.gr_open = int(gripper_open_val)
        self.gr_close = int(gripper_close_val)
        self.gr_speed = int(gripper_speed)
        self.ANCHOR_PY = list(ANCHOR_PY)

    def move_and_wait(self, mode: str, value: Sequence[float], speed: Optional[int]=None,
                    delay: float=0.5, poll_sec: float=0.1):
        """
        비동기 명령 후 is_moving()으로 정지까지 대기.
        - mode = 'coords' : send_coords(value, speed, mode=0)
        - mode = 'angles' : send_angles(value, speed)
        """
        try:
            spd = speed or self.move_speed
            if mode == "coords":
                self.mc.send_coords(list(value), spd, 0)
            elif mode == "angles":
                self.mc.send_angles(list(value), spd)
            else:
                raise ValueError("mode must be 'coords' or 'angles'")
            self.wait_until_stop(poll_sec=poll_sec, extra_delay=delay)
        except Exception as e:
            print(f"[WARN] move_and_wait 실패: {e}")


    # ── 전원/기본 이동 ─────────────────────────────
    def power_on(self, go_anchor: bool=True):
        try:
            self.mc.power_on()
            # 펌웨어/모델에 따라 미지원일 수 있어 예외 안전
            try: self.mc.set_fresh_mode(0)
            except Exception: pass
            try: self.mc.set_gripper_mode(0)
            except Exception: pass
            try: self.mc.init_electric_gripper()
            except Exception: pass
            try: self.mc.set_electric_gripper(0)
            except Exception: pass

            if go_anchor:
                self.go_anchor()
        except Exception as e:
            print(f"[WARN] power_on 실패: {e}")

    def power_off(self):
        try:
            self.mc.power_off()
        except Exception as e:
            print(f"[WARN] power_off 실패: {e}")

    def stop(self):
        try:
            self.mc.stop()
        except Exception as e:
            print(f"[WARN] stop 실패: {e}")

    def wait_until_stop(self, poll_sec: float=0.1, extra_delay: float=0.0, timeout: Optional[float]=None):
        """
        send_* 호출 후 모션 종료까지 대기.
        timeout 지정 시 해당 시간 초과하면 반환.
        """
        start = time.time()
        try:
            while True:
                mv = self.mc.is_moving()
                if mv in (0, -1):   # 0:not moving, -1:error
                    break
                time.sleep(poll_sec)
                if timeout is not None and (time.time() - start) > timeout:
                    print("[WARN] wait_until_stop: timeout")
                    break
        except Exception as e:
            print(f"[WARN] is_moving 폴링 실패: {e}")
        if extra_delay > 0:
            time.sleep(extra_delay)

    # ── 좌표/자세 이동 ─────────────────────────────
    def move_coords(self, coords: Sequence[float], speed: Optional[int]=None, mode: int=0, sleep: float=0.0):
        """동기 이동 (도달 보장)"""
        try:
            self.mc.sync_send_coords(list(coords), speed or self.move_speed, mode)
            if sleep > 0:
                time.sleep(sleep)
        except Exception as e:
            print(f"[WARN] sync_send_coords 실패: {e}")

 

    # ── 그리퍼 호환 래퍼 ──────────────────────────
    def gripper_open(self):
        try:
            self.mc.set_gripper_value(self.gr_open, self.gr_speed, 1)
        except Exception as e:
            print(f"[WARN] gripper open 실패: {e}")

    def gripper_close(self):
        try:
            self.mc.set_gripper_value(self.gr_close, self.gr_speed, 1)
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
                self.mc.set_gripper_value(40, 20, 1)
                # 이탈 자세
                self.move_and_wait("angles", [-22.23, 0, 78.95, -21, -87.36, -15])

            elif val == 'blue':
                z = z + 170
                self.move_and_wait("angles", [12.12, 0, 70.83, -16.08, -67.5, -150])
                self.move_and_wait("coords", [-219.2, -281.9, z, 179.41, -5, -100], speed=10, delay=1.0)
                self.mc.set_gripper_value(40, 20, 1)
                self.move_and_wait("angles", [12.12, 0, 70.83, -16.08, -67.5, -150])

            elif val in ('red'):
                z = z + 160
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])
                self.move_and_wait("coords", [-180, 240, z, -173.15, 0, 90], speed=10, delay=1.0)
                self.mc.set_gripper_value(40, 20, 1)
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])

            elif val in ('anomaly'):
                self.move_and_wait("angles", [-80, -20, -65, -0, 90, -90])
                self.mc.set_gripper_value(40, 20, 1)
              

            else:
                print(f"[WARN] place_box: 알 수 없는 색상 '{val}' (동작 생략)")
                return

            # # 공통 복귀 시퀀스
            self.go_anchor()

        except Exception as e:
            print(f"[ERROR] place_box 실패: {e}")

    # ── 편의 함수 ─────────────────────────────────
    def go_anchor(self, speed=20, delay=0.2):
        try:
            # 웨이포인트 먼저 이동
            self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25], speed, delay)
            """앵커 좌표로 복귀"""
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] go_anchor 실패: {e}")

    def refresh_home(self, speed=20, delay=0.2):
        try:
            self.move_and_wait("angles", [0, 10, -80, -0, 90, -90], speed, delay)
            """앵커 좌표로 복귀"""
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] go_anchor 실패: {e}")

       


   