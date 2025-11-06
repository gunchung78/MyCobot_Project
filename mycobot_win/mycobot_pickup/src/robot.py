# -*- coding: utf-8 -*-
import time
from pymycobot.mycobot320 import MyCobot320
import config as C

class Robot:
    def __init__(self, port, baud, move_speed,
                 gripper_open_val, gripper_close_val, gripper_speed):
        self.mc = MyCobot320(port, baud)
        self.move_speed = move_speed
        self.gr_open = gripper_open_val
        self.gr_close = gripper_close_val
        self.gr_speed = gripper_speed

    # ── 전원/기본 이동 ─────────────────────────────
    def power_on(self):
        try:
            self.mc.power_on()
            self.mc.set_fresh_mode(0)
            self.mc.set_gripper_mode(0)
            self.mc.init_electric_gripper()
            self.mc.set_electric_gripper(0)
            self.mc.sync_send_coords(C.ANCHOR_PY, 20)
            time.sleep(1)
        except Exception as e:
            print(f"[WARN] power_on 실패: {e}")

    def move_coords(self, coords, speed=None, mode=0, sleep=0):
        try:
            self.mc.sync_send_coords(coords, speed or self.move_speed, mode)
            time.sleep(sleep)
        except Exception as e:
            print(f"[WARN] send_coords 실패: {e}")

    # ── 그리퍼 호환 래퍼 ──────────────────────────
    def gripper_open(self):
        try:
            if hasattr(self.mc, "set_electric_gripper"):
                self.mc.set_electric_gripper(0)
            if hasattr(self.mc, "set_gripper_value"):
                self.mc.set_gripper_value(self.gr_open, self.gr_speed, 1)
            elif hasattr(self.mc, "set_gripper_state"):
                self.mc.set_gripper_state(0, self.gr_speed)
        except Exception as e:
            print(f"[WARN] gripper open failed: {e}")

    def gripper_close(self):
        try:
            if hasattr(self.mc, "set_gripper_value"):
                self.mc.set_gripper_value(self.gr_close, self.gr_speed, 1)
            elif hasattr(self.mc, "set_gripper_state"):
                self.mc.set_gripper_state(1, self.gr_speed)
            if hasattr(self.mc, "set_electric_gripper"):
                self.mc.set_electric_gripper(1)
        except Exception as e:
            print(f"[WARN] gripper close failed: {e}")
