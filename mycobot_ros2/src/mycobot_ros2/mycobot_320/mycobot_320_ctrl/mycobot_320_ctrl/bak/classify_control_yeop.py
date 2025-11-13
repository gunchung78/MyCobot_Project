#!/usr/bin/env python3
import time
import math
import traceback
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from mycobot_interfaces.srv import SetAngles, SetCoords, GripperStatus
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mycobot_320_ctrl.src.config_loader import load_config

# ============================================
# ✅ 설정 및 상수
# ============================================
try:
    from mycobot_320_ctrl.src.ros_robot123 import ROS_Robot
    print("[INFO] ros_robot successfully imported.")
except ImportError:
    print("[WARN] ros_robot import failed, using default SEQUENCE_LIST.")
    SEQUENCE_LIST = [
        ['angles', [5.0, 0.0, 5.0, 0.0, 5.0, 0.0]],
        ['angles', [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25]],
        # ['coords', [-230.0, -50.0, 300.0, -180.0, 0.0, 0.0]],
        ['angles', [-10.0, 0.0, 78.95, -21.0, -87.36, -15.0]],
        # ['coords', [-293.5, -25, 148, -176, 0, 90]],
        ['angles', [0.0, 0.0, -80.0, -0.0, 90.0, -90.0]]
    ]

JOINT_LIMITS = [180.0] * 6
C = load_config("config.json")


# ============================================
# ✅ ClassifyControl 클래스
# ============================================
class ClassifyControl(Node):
    def __init__(self):
        super().__init__('classify_control')

        # --- 파라미터 ---
        self.declare_parameter('test_mode', False)
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().bool_value
        self.get_logger().info(f'🧩 test_mode: {self.test_mode}')

        # --- 서비스 클라이언트 ---
        self.client_angles = self.create_client(SetAngles, 'set_angles')
        self.client_coords = self.create_client(SetCoords, 'set_coords')
        self.client_gripper = self.create_client(GripperStatus, 'set_gripper')

        if not self.test_mode:
            while not (self.client_angles.wait_for_service(timeout_sec=2.0)
                       and self.client_coords.wait_for_service(timeout_sec=2.0)
                       and self.client_gripper.wait_for_service(timeout_sec=2.0)):
                self.get_logger().info('⏳ Waiting for /set_angles, /set_coords, /set_gripper ...')

        # --- 상태 변수 ---
        self._lock = threading.Lock()
        self.current_angles = None
        self.coords_xyzrpy = None
        self.current_stamp = None
        self.target_angles = None
        self.target_coords = None
        self.reached_event = threading.Event()
        self.result_list = SEQUENCE_LIST

        # --- QoS ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- 구독 ---
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, qos_profile)
        self.create_subscription(Float32MultiArray, '/mycobot/coords', self._on_coords, qos_profile)
        self.create_subscription(Float32MultiArray, '/mycobot/angles', self._on_angles, qos_profile)

        # --- 제어 파라미터 ---
        self.speed = 25
        self.reach_timeout = 15.0
        self.check_period = 0.1
        self.consecutive_ok_needed = 2
        self.stale_after_sec = 10.0

        # --- 허용치 ---
        self.tol_joint_deg = 5.0
        self.tol_xyz_mm = 5.0
        self.tol_rpy_deg = 5.0

        # --- 모니터 스레드 ---
        self._run = True
        self._mon = threading.Thread(target=self._monitor_loop, daemon=True)
        self._mon.start()

        self.get_logger().info('✅ ClassifyControl initialized.')
        threading.Thread(target=self._run_sequence, args=(self.result_list,), daemon=True).start()

    # ============================================
    # 📡 콜백 함수
    # ============================================
    def _on_joint_state(self, msg: JointState):
        if self.test_mode:
            return
        if len(msg.position) == 6:
            with self._lock:
                self.current_angles = [math.degrees(v) for v in msg.position]
                # self.current_stamp = msg.header.stamp if msg.header.stamp.sec > 0 else self.get_clock().now().to_msg()
                self.current_stamp = self.get_clock().now().to_msg()

    def _on_coords(self, msg: Float32MultiArray):
        if not self.test_mode and len(msg.data) == 6:
            with self._lock:
                self.coords_xyzrpy = [float(v) for v in msg.data]
                self.current_stamp = self.get_clock().now().to_msg()

    def _on_angles(self, msg: Float32MultiArray):
        if not self.test_mode and len(msg.data) == 6:
            with self._lock:
                self.current_angles = [float(v) for v in msg.data]
                self.current_stamp = self.get_clock().now().to_msg()

    # ============================================
    # 🛠 공통 유틸 함수
    # ============================================
    def _clamp_deg(self, arr_deg):
        return [max(-lim, min(lim, float(a))) for a, lim in zip(arr_deg, JOINT_LIMITS)]

    def _call_service_sync(self, client, req, name, timeout=8.0):
        """공통 동기 서비스 호출 래퍼"""
        try:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            result = future.result()
            if result and getattr(result, 'flag', True):
                self.get_logger().info(f'✅ {name} success.')
                return True
            else:
                self.get_logger().warn(f'⚠️ {name} failed or timeout.')
                return False
        except Exception:
            self.get_logger().error(traceback.format_exc())
            return False

    # ============================================
    # 🚀 동작 명령
    # ============================================
    def _send_set_angles(self, angles_deg):
        if self.test_mode:
            threading.Thread(target=self._sim_move_angles, args=(angles_deg,), daemon=True).start()
            self.get_logger().info(f'🧪 [SIM] MoveAngles: {angles_deg}')
            return True

        req = SetAngles.Request()
        req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6 = self._clamp_deg(angles_deg)
        req.speed = int(self.speed)
        return self._call_service_sync(self.client_angles, req, "SetAngles")

    def _send_set_coords(self, xyzrpy):
        if self.test_mode:
            threading.Thread(target=self._sim_move_coords, args=(xyzrpy,), daemon=True).start()
            self.get_logger().info(f'🧪 [SIM] MoveCoords: {xyzrpy}')
            return True

        req = SetCoords.Request()
        req.x, req.y, req.z, req.rx, req.ry, req.rz = map(float, xyzrpy)
        req.speed = int(self.speed)
        req.model = 1
        return self._call_service_sync(self.client_coords, req, "SetCoords")

    def _send_set_gripper(self, status, speed=80):
        if self.test_mode:
            self.get_logger().info(f'🧪 [SIM] Gripper {"Open" if status else "Close"}')
            return True

        req = GripperStatus.Request()
        req.status = bool(status)
        success = self._call_service_sync(self.client_gripper, req, "SetGripper")
        return success

    # ============================================
    # 🧩 동기 이동 함수
    # ============================================
    def move_joint(self, target_deg):
        with self._lock:
            self.target_angles = target_deg[:]
            self.target_coords = None
        self.reached_event.clear()
        if self._send_set_angles(target_deg):
            self._wait_until_reached("angles")

    def move_coords(self, xyzrpy):
        with self._lock:
            self.target_coords = xyzrpy[:]
            self.target_angles = None
        self.reached_event.clear()
        if self._send_set_coords(xyzrpy):
            self._wait_until_reached("coords")

    def _wait_until_reached(self, mode):
        t0 = time.time()
        while time.time() - t0 < self.reach_timeout:
            if self.reached_event.wait(timeout=self.check_period):
                self.get_logger().info(f'✅ Reached {mode} target.')
                self.reached_event.clear()
                return True
        self.get_logger().warn(f'⌛ Timeout waiting for {mode}.')
        return False

    # ============================================
    # 🧠 모니터 스레드
    # ============================================
    def _monitor_loop(self):
        ok_count = 0
        while self._run:
            time.sleep(self.check_period)
            with self._lock:
                cur_a = list(self.current_angles) if self.current_angles else None
                cur_c = list(self.coords_xyzrpy) if self.coords_xyzrpy else None
                tgt_a = list(self.target_angles) if self.target_angles else None
                tgt_c = list(self.target_coords) if self.target_coords else None
                stamp = self.current_stamp

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            st_sec = (stamp.sec + stamp.nanosec * 1e-9) if stamp else 0.0
            delta = now_sec - st_sec

            # 1️⃣ 타임스탬프 상태
            if (delta) > self.stale_after_sec:
                self.get_logger().warn(f"[MON] 🕒 Data stale ({delta:.2f}s old) → reset ok_count")
                ok_count = 0
                continue

            reached = False

            # 2️⃣ 각도 기반 도달 판단
            if tgt_a and cur_a and len(cur_a) == 6:
                diffs = [abs(c - t) for c, t in zip(cur_a, tgt_a)]
                max_diff = max(diffs)
                self.get_logger().info(
                    f"[MON] mode=angles | cur={list(map(lambda x: round(x,1), cur_a))} | tgt={list(map(lambda x: round(x,1), tgt_a))} | diffs={[round(d,2) for d in diffs]} | max={max_diff:.2f}"
                )
                reached = (max_diff <= self.tol_joint_deg)

            # 3️⃣ 좌표 기반 도달 판단
            elif tgt_c and cur_c and len(cur_c) == 6:
                d_xyz = [abs(c - t) for c, t in zip(cur_c[:3], tgt_c[:3])]
                d_rpy = [abs(c - t) for c, t in zip(cur_c[3:], tgt_c[3:])]
                self.get_logger().info(
                    f"[MON] mode=coords | d_xyz={[round(d,2) for d in d_xyz]} | d_rpy={[round(d,2) for d in d_rpy]}"
                )
                reached = (all(d <= self.tol_xyz_mm for d in d_xyz) and
                        all(d <= self.tol_rpy_deg for d in d_rpy))
            else:
                # 대상 미지정
                continue

            # 4️⃣ 결과 반영
            if reached:
                ok_count += 1
                self.get_logger().info(f"[MON] ✅ Step OK ({ok_count}/{self.consecutive_ok_needed})")
                if ok_count >= self.consecutive_ok_needed:
                    self.get_logger().info(f"[MON] 🎯 Reached target → event set()")
                    self.reached_event.set()
                    ok_count = 0
            else:
                if ok_count > 0:
                    self.get_logger().info(f"[MON] ❌ Reset ok_count (was {ok_count})")
                ok_count = 0

    # ============================================
    # 🎯 시퀀스 실행
    # ============================================
    def _run_sequence(self, seq=None):
        if not seq:
            self.get_logger().warn("⚠️ 시퀀스 데이터가 비어있습니다. 동작 생략.")
            return

        self.get_logger().info(f'🎯 Running sequence ({len(seq)} steps)')
        for i, step in enumerate(seq, 1):
            try:
                mode, data = step[0].lower(), step[1]
                self.get_logger().info(f'➡️ Step {i}/{len(seq)} | {mode}: {data}')

                if mode == "angles":
                    self.move_joint(data)
                elif mode == "coords":
                    self.move_coords(data)
                elif mode == "grasp":
                    self._send_set_gripper(int(data[0]), int(data[1]) if len(data) > 1 else 80)
                    time.sleep(1.0)
                else:
                    self.get_logger().warn(f'❓ Unknown mode: {mode}')
            except Exception:
                self.get_logger().error(traceback.format_exc())
        self.get_logger().info('✅ Sequence finished.')

    # ============================================
    # 🧪 시뮬레이터
    # ============================================
    def _sim_move_angles(self, target_deg):
        steps, dt = max(10, int(30 * (self.speed / 100.0))), 0.03
        with self._lock:
            cur = self.current_angles[:] if self.current_angles else [0.0] * 6
        target = self._clamp_deg(target_deg)
        for s in range(1, steps + 1):
            alpha = s / steps
            newv = [(1 - alpha) * c + alpha * t for c, t in zip(cur, target)]
            with self._lock:
                self.current_angles = newv
                self.current_stamp = self.get_clock().now().to_msg()
            time.sleep(dt)

    def _sim_move_coords(self, xyzrpy):
        steps, dt = max(10, int(30 * (self.speed / 100.0))), 0.03
        with self._lock:
            cur = self.coords_xyzrpy[:] if self.coords_xyzrpy else [0.0, 0.0, 250.0, -180.0, 0.0, 0.0]
        target = list(map(float, xyzrpy))
        for s in range(1, steps + 1):
            alpha = s / steps
            newv = [(1 - alpha) * c + alpha * t for c, t in zip(cur, target)]
            with self._lock:
                self.coords_xyzrpy = newv
                self.current_stamp = self.get_clock().now().to_msg()
            time.sleep(dt)

    # ============================================
    # 🔚 종료 처리
    # ============================================
    def destroy_node(self):
        self._run = False
        try:
            self._mon.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClassifyControl()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('🧩 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
