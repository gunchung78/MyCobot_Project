#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mycobot_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles

# ===== 사용자 계획 예시 =====
PLAN = [
    ('angles', [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25]),
    ('angles', [0, 0, -80, 0, 90, -90]),
    ('gripper', True),   # open (기존 ('graps', [60,20]) 대체)
    ('angles', [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25]),
    ('coords', [216.8, -166.9, 243.0, -180.0, 0.0, 1.1093331416329306]),
    ('gripper', False),  # close (기존 ('graps', [10,20]) 대체)
    ('angles', [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25]),
    ('angles', [12.12, 0, 70.83, -16.08, -67.5, -150]),
    ('coords', [-219.2, -281.9, 170, 179.41, -5, -100]),
    ('gripper', True),   # open
    ('angles', [12.12, 0, 70.83, -16.08, -67.5, -150]),
    ('angles', [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25]),
    ('angles', [0, 0, -80, 0, 90, -90]),
]


# 좌표 안전 한계(필요시 조정)
COORD_LIMITS = {
    'x':  (-350, 350),
    'y':  (-350, 350),
    'z':  (-41, 523.9),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180),
}

DEFAULT_SPEED = 30  # 0~100
DEFAULT_MODEL = 0   # 0:moveJ, 1:moveL, 2:moveC (드라이버/라이브러리 기준)


class PlanRunner(Node):
    def __init__(self):
        super().__init__('plan_runner')

        # --- service clients (listen_real_service.py 가 띄워져 있어야 함) ---
        self.cli_angles  = self.create_client(SetAngles,  '/set_angles')
        self.cli_coords  = self.create_client(SetCoords,  '/set_coords')
        self.cli_gripper = self.create_client(GripperStatus, '/set_gripper')
        self.cli_get_c   = self.create_client(GetCoords,  '/get_coords')
        self.cli_get_a   = self.create_client(GetAngles,  '/get_angles')

        # 서비스 대기
        for cli, name in [(self.cli_angles,'/set_angles'),
                          (self.cli_coords,'/set_coords'),
                          (self.cli_gripper,'/set_gripper'),
                          (self.cli_get_c,'/get_coords'),
                          (self.cli_get_a,'/get_angles')]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'⏳ waiting for {name} ...')

    # ---------- helpers ----------
    def _spin_until(self, future, timeout_sec=10.0):
        """future 완료 대기. 시간 초과 시 False 반환"""
        start = self.get_clock().now()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start) > Duration(seconds=timeout_sec):
                return False
        return future.result() is not None

    def _check_coords_limits(self, coords):
        (x,y,z,rx,ry,rz) = coords
        axes = ['x','y','z','rx','ry','rz']
        for i, ax in enumerate(axes):
            lo, hi = COORD_LIMITS[ax]
            if coords[i] < lo or coords[i] > hi:
                self.get_logger().error(f"[coords] {ax}={coords[i]} out of range [{lo},{hi}]")
                return False
        return True

    # ---------- service wrappers ----------
    def call_angles(self, angles, speed=DEFAULT_SPEED):
        req = SetAngles.Request()
        (req.joint_1, req.joint_2, req.joint_3,
         req.joint_4, req.joint_5, req.joint_6) = angles
        req.speed = int(speed)
        fut = self.cli_angles.call_async(req)
        ok = self._spin_until(fut, timeout_sec=20.0)
        if not ok or not getattr(fut.result(), 'flag', False):
            self.get_logger().error(f"[angles] failed: {angles}")
            return False
        return True

    def call_coords(self, coords, speed=DEFAULT_SPEED, model=DEFAULT_MODEL):
        if not self._check_coords_limits(coords):
            return False
        req = SetCoords.Request()
        (req.x, req.y, req.z, req.rx, req.ry, req.rz) = coords
        req.speed = int(speed)
        req.model = int(model)
        fut = self.cli_coords.call_async(req)
        ok = self._spin_until(fut, timeout_sec=25.0)
        if not ok or not getattr(fut.result(), 'flag', False):
            self.get_logger().error(f"[coords] failed: {coords}")
            return False
        return True

    # 기존 call_grasp(value, speed) 제거하고 ↓로 교체
    def call_gripper(self, status):
        """
        status: bool 또는 문자열 ('open'/'close' 대소문자 무관)
        """
        if isinstance(status, str):
            s = status.strip().lower()
            if s in ('open', 'o', 'true', '1'):
                status_bool = True
            elif s in ('close', 'c', 'false', '0'):
                status_bool = False
            else:
                self.get_logger().error(f"[gripper] invalid string: {status}")
                return False
        else:
            status_bool = bool(status)

        req = GripperStatus.Request()
        req.status = status_bool  # True=open, False=close
        fut = self.cli_gripper.call_async(req)
        ok = self._spin_until(fut, timeout_sec=10.0)
        if not ok or not getattr(fut.result(), 'flag', False):
            self.get_logger().error(f"[gripper] failed: {status}")
            return False
        return True


    # ---------- main ----------
    def run_plan(self, plan):
        """
        plan: list of tuples
          ('angles', [a1..a6]) or
          ('coords', [x,y,z,rx,ry,rz]) or
          ('grasp'/'graps', [value, speed])
        """
        step_ok = True
        for idx, item in enumerate(plan, start=1):
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                self.get_logger().error(f"[step {idx}] invalid item format: {item}")
                return False

            cmd, data = item[0], item[1]
            try:
                if cmd == 'angles':
                    assert isinstance(data, (list, tuple)) and len(data) == 6
                    self.get_logger().info(f"[{idx}] angles -> {data}")
                    step_ok = self.call_angles(list(map(float, data)))
                elif cmd == 'coords':
                    assert isinstance(data, (list, tuple)) and len(data) == 6
                    self.get_logger().info(f"[{idx}] coords -> {data}")
                    step_ok = self.call_coords(list(map(float, data)))
                elif cmd in ('gripper',):
                    # data가 bool 또는 str이어야 함
                    self.get_logger().info(f"[{idx}] gripper -> {data}")
                    step_ok = self.call_gripper(data)
                else:
                    self.get_logger().error(f"[step {idx}] unknown command: {cmd}")
                    return False
            except Exception as e:
                self.get_logger().error(f"[step {idx}] exception: {e}")
                return False

            if not step_ok:
                self.get_logger().error(f"[step {idx}] FAILED, abort sequence.")
                return False

            # 동작 안정화를 위한 소폭 대기(필요시 조절)
            time.sleep(0.2)

        self.get_logger().info("✅ All steps completed successfully.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PlanRunner()
    try:
        node.run_plan(PLAN)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
