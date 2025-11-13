#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import traceback
from typing import Optional, List, Union

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32MultiArray
from mycobot_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles

# === 네 환경에 맞게 경로 수정 ===
from mycobot_320_ctrl.src.config_loader import load_config            # ../config.json 로드
from mycobot_320_ctrl.src.ros_robot import ROS_Robot          # <<< ROS_Robot 클래스를 제공하는 실제 모듈로 교체
import threading, queue
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

COORD_LIMITS = {
    'x':  (-350, 350),
    'y':  (-350, 350),
    'z':  (-41, 523.9),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180),
}

DEFAULT_SPEED = 30
DEFAULT_MODEL = 0  # 드라이버 기본(참고용)


def map_color(code: float) -> Optional[str]:
    # 0=red, 1=blue, 2=green
    iv = int(round(code))
    return {0: 'red', 1: 'blue', 2: 'green'}.get(iv, None)

def map_detected(code: float) -> Optional[str]:
    # 0=normal, 1=anomaly
    iv = int(round(code))
    return {0: 'normal', 1: 'anomaly'}.get(iv, None)


class ClassifyControl(Node):
    """
    /classify_input(Float32MultiArray): [x_t, y_t, rz_t, color_code, detected_code]
    → ROS_Robot(...)로 plan 생성 → /set_angles /set_coords /set_gripper 순차 실행
    """

    def __init__(self):
        super().__init__('classify_control')
        
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.RELIABLE)

        # --- 서비스 클라이언트 ---
        self.cli_angles  = self.create_client(SetAngles,  '/set_angles')
        self.cli_coords  = self.create_client(SetCoords,  '/set_coords')
        self.cli_gripper = self.create_client(GripperStatus, '/set_gripper')
        self.cli_get_c   = self.create_client(GetCoords,  '/get_coords')
        self.cli_get_a   = self.create_client(GetAngles,  '/get_angles')

        self._q = queue.Queue()
        self._stop = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        for cli, name in [(self.cli_angles,'/set_angles'),
                          (self.cli_coords,'/set_coords'),
                          (self.cli_gripper,'/set_gripper'),
                          (self.cli_get_c,'/get_coords'),
                          (self.cli_get_a,'/get_angles')]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'⏳ waiting for {name} ...')

        # --- 단일 구독: [x_t, y_t, rz_t, color_code, detected_code] ---
        self.sub = self.create_subscription(
            Float32MultiArray, '/classify_input', self._cb_input, qos
        )

        # 최신 입력
        self._x_t: Optional[float] = None
        self._y_t: Optional[float] = None
        self._rz_t: Optional[float] = None
        self._color: Optional[str] = None
        self._detected_type: Optional[str] = None
        self.plan = []

        self._busy = False

        # 설정 로드
        try:
            self.C = load_config("../config.json")
        except Exception:
            self.get_logger().warn("load_config('../config.json') 실패 → 'config.json' 재시도")
            self.C = load_config("config.json")

        self.get_logger().info("ClassifyControl ready. Waiting /classify_input ...")

    # ========== Subscriber ==========
    def _cb_input(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)
            if len(data) < 5:
                self.get_logger().warn(f"/classify_input length<5: {data}")
                return

            x_t, y_t, rz_t, color_code, detected_code = map(float, data[:5])
            self._x_t, self._y_t, self._rz_t = x_t, y_t, rz_t
            self._color = map_color(color_code)
            self._detected_type = map_detected(detected_code)

            self.get_logger().info(
                f"in: x={x_t:.2f}, y={y_t:.2f}, rz={rz_t:.2f}, color={self._color}, type={self._detected_type}"
            )
            self._try_run_once()

        except Exception:
            self.get_logger().error("input cb error:\n" + traceback.format_exc())

    # ========== Trigger ==========
    def _ready(self) -> bool:
        return (self._x_t is not None and
                self._y_t is not None and
                self._rz_t is not None and
                (self._color is not None or self._detected_type is not None))

    def _select_mode(self) -> int:
        # 색상 코드가 유효하면 color 기반(mode=1), 아니면 YOLO 기반(mode=0)
        return 1 if self._color is not None else 0

    def _try_run_once(self):
        if not self._ready():
            return

        # 중복 폭주를 막고 싶으면 큐 길이 상한을 둡니다(예: 2)
        if self._q.qsize() >= 2:
            self.get_logger().warn("enqueue skipped: queue full")
            return

        self.get_logger().info("🔶 Inputs ready. Building plan via ROS_Robot...")
        try:
            mode = 1
            bot = ROS_Robot(mode, self.C, self._x_t, self._y_t, self._rz_t,
                            self._color, self._detected_type)
            plan = bot.main_fow() or []
            if not plan:
                self.get_logger().warn("빈 plan 생성 → 실행 생략")
                return

            # ✅ 실행은 여기서 하지 말고 큐에 넣기
            self._q.put(plan)
            self.get_logger().info(f"▶ plan enqueued (len={len(plan)})")
        except Exception:
            self.get_logger().error("build plan error:\n" + traceback.format_exc())


    # ========== Call wrappers ==========
    def _spin_until(self, future, timeout_sec=15.0) -> bool:
        import time
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            time.sleep(0.05)   # executor는 메인 스레드가 돌림. 여기선 '기다리기'만!
            if time.time() > deadline:
                return False
        return future.result() is not None

    def _check_coords_limits(self, coords):
        axes = ['x','y','z','rx','ry','rz']
        for i, ax in enumerate(axes):
            lo, hi = COORD_LIMITS[ax]
            if coords[i] < lo or coords[i] > hi:
                self.get_logger().error(f"[coords] {ax}={coords[i]} out of range [{lo},{hi}]")
                return False
        return True

    def _call_angles(self, angles, speed=DEFAULT_SPEED) -> bool:
        req = SetAngles.Request()
        (req.joint_1, req.joint_2, req.joint_3,
         req.joint_4, req.joint_5, req.joint_6) = angles
        req.speed = int(speed)
        fut = self.cli_angles.call_async(req)
        ok = self._spin_until(fut, timeout_sec=25.0)
        return bool(ok and getattr(fut.result(), 'flag', False))

    def _call_coords(self, coords, speed=DEFAULT_SPEED, model=DEFAULT_MODEL) -> bool:
        if not self._check_coords_limits(coords):
            return False
        req = SetCoords.Request()
        (req.x, req.y, req.z, req.rx, req.ry, req.rz) = coords
        req.speed = int(speed)
        req.model = int(model)
        fut = self.cli_coords.call_async(req)
        ok = self._spin_until(fut, timeout_sec=30.0)
        return bool(ok and getattr(fut.result(), 'flag', False))

    def _call_gripper(self, status: bool) -> bool:
        req = GripperStatus.Request()
        req.status = bool(status)  # True=open, False=close
        fut = self.cli_gripper.call_async(req)
        ok = self._spin_until(fut, timeout_sec=10.0)
        return bool(ok and getattr(fut.result(), 'flag', False))

    # ========== Plan runner ==========
    def _run_plan(self, plan: List[List]) -> bool:
        """
        plan 형식(ROS_Robot 결과 그대로):
          ['angles', [a1..a6]]  or
          ['coords', [x,y,z,rx,ry,rz]]  or
          ['gripper', True/False]
        """
        for idx, item in enumerate(plan, start=1):
            if (not isinstance(item, (list, tuple))) or len(item) != 2:
                self.get_logger().error(f"[step {idx}] invalid item: {item}")
                return False

            cmd, data = item[0], item[1]
            try:
                if cmd == 'angles':
                    assert isinstance(data, (list, tuple)) and len(data) == 6
                    self.get_logger().info(f"[{idx}] angles -> {data}")
                    if not self._call_angles(list(map(float, data))):
                        self.get_logger().error(f"[{idx}] angles fail")
                        return False

                elif cmd == 'coords':
                    assert isinstance(data, (list, tuple)) and len(data) == 6
                    self.get_logger().info(f"[{idx}] coords -> {data}")
                    if not self._call_coords(list(map(float, data))):
                        self.get_logger().error(f"[{idx}] coords fail")
                        return False

                elif cmd == 'gripper':
                    self.get_logger().info(f"[{idx}] gripper -> {data}")
                    if not self._call_gripper(bool(data)):
                        self.get_logger().error(f"[{idx}] gripper fail")
                        return False

                else:
                    self.get_logger().error(f"[{idx}] unknown cmd: {cmd}")
                    return False

                time.sleep(0.2)

            except Exception:
                self.get_logger().error(f"[{idx}] exception:\n" + traceback.format_exc())
                return False
        return True
    
    def _worker_loop(self):
        while not self._stop:
            plan = self._q.get()
            try:
                ok = self._run_plan(plan)
                if not ok:
                    self.get_logger().error("plan failed")
                else:
                    self.get_logger().info("✅ plan finished")
            except Exception as e:
                self.get_logger().error(f"worker exception: {e}")
            finally:
                self._q.task_done()


def main(args=None):
    rclpy.init(args=args)
    node = ClassifyControl()
    try:
        exe = MultiThreadedExecutor(num_threads=2)
        exe.add_node(node)
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
