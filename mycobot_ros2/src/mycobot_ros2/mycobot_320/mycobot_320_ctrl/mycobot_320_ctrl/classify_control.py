#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading, queue
import traceback
from typing import Optional, List, Union

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32
from mycobot_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles

from mycobot_320_ctrl.src.config_loader import load_config           
from mycobot_320_ctrl.src.ros_robot import ROS_Robot                  

# 제어 파라미터 설정
COORD_LIMITS = {
    'x':  (-350, 350),
    'y':  (-350, 350),
    'z':  (-41, 523.9),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180),
}
ANGLES_SPEED = 47
COORDS_SPEED = 30
GRIPPER_TIME = 1.5
DEFAULT_MODEL = 0

def map_color(code: float) -> Optional[str]:
    # 1=red, 2=blue, 3=green, 그 외 None (detector.py의 color_map과 호환 주의!)
    iv = int(round(code))
    return {1: 'red', 2: 'blue', 3: 'green', 0: None}.get(iv, None)

def map_detected(code: float) -> Optional[str]:
    # 0=normal, 1=anomaly, 그 외 None
    iv = int(round(code))
    return {0: 'normal', 1: 'anomaly'}.get(iv, None)

class ClassifyControl(Node):
    """
    /detector_result(Float32MultiArray): [x_t, y_t, rz_t, color_code, detected_code]
    /classify_result(Int32): 0=normal, 1=anomaly, -1/기타=unknown
    → ROS_Robot(...)로 plan 생성 → /set_angles /set_coords /set_gripper 순차 실행
    """

    def __init__(self):
        super().__init__('classify_control')

        # --- 파라미터 설정 ---
        self.declare_parameter('detect_mode', 'detect_only')
        self._detect_mode = self.get_parameter('detect_mode').get_parameter_value().string_value
        
        # --- QoS (depth=1, RELIABLE) ---
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # --- 서비스 클라이언트 ---
        self.cli_angles  = self.create_client(SetAngles,  '/set_angles')
        self.cli_coords  = self.create_client(SetCoords,  '/set_coords')
        self.cli_get_c   = self.create_client(GetCoords,  '/get_coords')
        self.cli_get_a   = self.create_client(GetAngles,  '/get_angles')
        self.cli_gripper = self.create_client(GripperStatus, '/set_gripper')

        # --- 큐/스레드 ---
        self._q = queue.Queue()
        self._mtx = threading.Lock()         # 분류 결과/입력 상태 보호용
        self._busy = False                   # 실행 중이면 True
        self._stop = False
        
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # --- 서비스 준비 대기 ---
        for cli, name in [(self.cli_angles, '/set_angles'),
                          (self.cli_coords, '/set_coords'),
                          (self.cli_get_c,  '/get_coords'),
                          (self.cli_get_a,  '/get_angles'),
                          (self.cli_gripper,'/set_gripper')]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'⏳ waiting for {name} ...')

        # --- 구독 등록 ---
        # 1) 검출 결과: [x_t, y_t, rz_t, color_code, detected_code]
        self.sub_det = self.create_subscription(
            Float32MultiArray, '/detector_result', self._cb_input, qos
        )

        # 2) YOLO 분류 결과: Int32 (0 normal, 1 anomaly, 그 외 unknown)
        self.classify_result: int = -1  # 최신 값을 캐시
        self.sub_cls = self.create_subscription(
            Int32, '/classify_result', self._cb_classify, qos
        )

        # --- 변수선언 ---
        self._x_t: Optional[float] = None
        self._y_t: Optional[float] = None
        self._rz_t: Optional[float] = None
        self._color: Optional[str] = None
        self._detected_type: Optional[str] = None

        # --- 설정 로드 ---
        try:
            self.C = load_config("../config.json")
        except Exception:
            self.get_logger().warning("load_config('../config.json') 실패 → 'config.json' 재시도")
            self.C = load_config("config.json")

        self.get_logger().info("ClassifyControl ready. Waiting /detector_result & /classify_result ...")

        # --- 로봇 설정 ---
        self._bot = ROS_Robot(self._detect_mode, self.C)
        self._call_angles([0.0, 0.0, -80.0, 0.0, 90.0, -90.0])
        self._call_gripper(True)
        

    # ========== Subscribers ==========
    def _cb_classify(self, msg: Int32):
        # 최신 분류 상태 업데이트(0=normal, 1=anomaly, else unknown)
        with self._mtx:
            self.classify_result = int(msg.data)

    def _cb_input(self, msg: Float32MultiArray):
        try:
            # 실행 중이면 드롭(“실행 중 보냈을 때 2번 동작” 방지)
            if self._busy:
                return

            data = list(msg.data)
            if len(data) < 5:
                self.get_logger().warning(f"/detector_result length<5: {data}")
                return

            x_t, y_t, rz_t, color_code, detected_code = map(float, data[:5])
            # 최신 YOLO 분류값을 가져와서 필요시 덮어쓰기
            with self._mtx:
                cls_val = self.classify_result

            # detector_result의 detected_code가 -1(unknown) 이거나
            # 항상 YOLO 분류를 우선하고 싶다면 아래처럼 교체:
            if int(round(detected_code)) not in (0, 1) and cls_val in (0, 1):
                detected_code = float(cls_val)

            # 상태 저장
            self._x_t, self._y_t, self._rz_t = x_t, y_t, rz_t
            self._color = map_color(color_code)
            self._detected_type = map_detected(detected_code)

            self.get_logger().info(
                f"in: x={x_t:.2f}, y={y_t:.2f}, rz={rz_t:.2f}, "
                f"color={self._color}, type={self._detected_type}, cls={cls_val}"
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

    # ========== Main ==========
    def _try_run_once(self):
        if not self._ready():
            return
        
        # 이미 실행 중이거나 큐에 작업 있으면 드랍(필요 시 정책 조정)
        if self._busy:
            self.get_logger().warning("enqueue skipped: busy")
            return
        if self._q.qsize() >= 1:
            self.get_logger().warning("enqueue skipped: queue has pending job")
            return
        self.get_logger().info("🔶 Inputs ready. Building plan via ROS_Robot...")
        try:
            rz = self._rz_t
            if rz is not None and abs(rz) < 1e-6:
                rz = None

            plan = self._bot.main_fow(self._x_t, self._y_t, rz,
                            self._color, self._detected_type) or []
            if not plan:
                self.get_logger().warning("빈 plan 생성 → 실행 생략")
                return

            # 실행은 워커에게 맡김
            self._busy = True
            self._q.put(plan)
            self.get_logger().info(f"▶ plan enqueued (len={len(plan)})")
        except Exception:
            self.get_logger().error("build plan error:\n" + traceback.format_exc())

    # ========== Service call helpers ==========
    def _spin_until(self, future, timeout_sec=15.0) -> bool:
        # 워커 스레드: spin() 사용 금지 → 폴링으로 완료만 확인
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
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

    def _call_angles(self, angles, speed=ANGLES_SPEED) -> bool:
        req = SetAngles.Request()
        (req.joint_1, req.joint_2, req.joint_3,
         req.joint_4, req.joint_5, req.joint_6) = angles
        req.speed = int(speed)
        fut = self.cli_angles.call_async(req)
        ok = self._spin_until(fut, timeout_sec=25.0)
        return bool(ok and getattr(fut.result(), 'flag', False))

    def _call_coords(self, coords, speed=COORDS_SPEED, model=DEFAULT_MODEL) -> bool:
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
        time.sleep(GRIPPER_TIME) 
        req = GripperStatus.Request()
        req.status = bool(status)  # True=open, False=close
        fut = self.cli_gripper.call_async(req)
        ok = self._spin_until(fut, timeout_sec=10.0)
        return bool(ok and getattr(fut.result(), 'flag', False))

    # ========== Plan runner & worker ==========
    def _run_plan(self, plan: List[List]) -> bool:
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
                self.get_logger().info("🏁 worker picked a plan")
                ok = self._run_plan(plan)
                if not ok:
                    self.get_logger().error("plan failed")
                else:
                    self.get_logger().info("✅ plan finished")
            except Exception as e:
                self.get_logger().error(f"worker exception: {e}")
            finally:
                # 실행 종료 → 새 입력 허용
                self._busy = False
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
