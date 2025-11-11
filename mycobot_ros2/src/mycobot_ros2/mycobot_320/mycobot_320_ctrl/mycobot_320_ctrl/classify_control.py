#!/usr/bin/env python3
import time
import math
import traceback
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from mycobot_interfaces.srv import SetAngles
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mycobot_320_ctrl.src.config_loader import load_config 


JOINT_LIMITS = [170.0, 160.0, 160.0, 170.0, 170.0, 175.0]


class ClassifyControl(Node):
    def __init__(self):
        super().__init__('classify_control')

        # --- 서비스 클라이언트 ---
        self.client_angles = self.create_client(SetAngles, 'set_angles')
        while not self.client_angles.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for /set_angles service...')

        # --- 상태 변수 ---
        self._lock = threading.Lock()
        self.current_angles_rad = None
        self.current_stamp = None
        self.target_angles_rad = None
        self.reached_event = threading.Event()
        self.coords_xyzrpy = None     # [x,y,z,rx,ry,rz]
        self.angles_deg = None        # [j1..j6] in degree

        # --- QoS 설정 ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- 구독 ---
        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, qos_profile
        )
        self.sub_classify = self.create_subscription(
            Int32, '/classify_result', self._on_classify, 10
        )
        self.sub_detector = self.create_subscription(
            Float32MultiArray, '/detector_result', self._on_detector, 10
        )
        self.sub_coords = self.create_subscription(
            Float32MultiArray, '/mycobot/coords', self._on_coords, qos_profile
        )
        self.sub_angles = self.create_subscription(
            Float32MultiArray, '/mycobot/angles', self._on_angles, qos_profile
        )

        # --- 제어 파라미터 ---
        self.speed = 25
        self.reach_tolerance_rad = math.radians(3.0)  # ⬇️ 오차 허용 조금 완화
        self.reach_timeout = 10.0
        self.check_period = 0.05
        self.consecutive_ok_needed = 3
        self.stale_after_sec = 3.0
        self.is_busy = False
        self.label = -1

        # --- 시퀀스 ---
        self.home_pose = [0, 0, 0, 0, 0, 0]
        self.good_sequence = [
            [0, 0, 0, 0, 0, 0],
            [10, -20, 30, 0, 0, 0],
            [20, -40, 50, 0, 0, 0],
            [20, -30, 40, 0, 0, 0],
            [30, 10, 20, 0, 0, 0],
        ]
        self.bad_sequence = [
            [0, 0, 0, 0, 0, 0],
            [-10, -20, 30, 0, 0, 0],
            [-20, -40, 50, 0, 0, 0],
            [-20, -30, 40, 0, 0, 0],
            [-30, 10, 20, 0, 0, 0],
        ]

        # --- 모니터 스레드 ---
        self._run = True
        self._mon = threading.Thread(target=self._monitor_loop, daemon=True)
        self._mon.start()

        # --- 첫 joint_states 대기 ---
        self.get_logger().info('✅ ClassifyControl ready. Waiting for first /joint_states...')
        if not self._wait_first_joint_states(timeout=5.0):
            self.get_logger().warn('⚠️ No /joint_states yet, continuing anyway.')

        self.get_logger().info('🏠 Moving to Home Pose...')
        self.move_joint(self.home_pose)

    # =========================================================
    # 🟢 joint_states 콜백
    def _on_joint_state(self, msg: JointState):
        with self._lock:
            self.current_angles_rad = list(msg.position)
            # 🟩 수정: stamp가 비정상일 때 현재 시간으로 대체
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                msg.header.stamp = self.get_clock().now().to_msg()
            self.current_stamp = msg.header.stamp

    # =========================================================
    # 🟢 분류 결과 콜백
    def _on_classify(self, msg: Int32):
        self.label = int(msg.data)

    # =========================================================
    # 🟢 detector 결과 콜백
    def _on_detector(self, msg: Float32MultiArray):
        if len(msg.data) == 4:
            result = msg.data

        if self.is_busy:
            self.get_logger().warn('⛔ Sequence busy, ignoring trigger.')
            return

        def _runner(seq):
            try:
                self.is_busy = True
                self.get_logger().info(seq)
            except Exception:
                self.get_logger().error(traceback.format_exc())
            finally:
                self.is_busy = False
        threading.Thread(target=_runner, args=(result,), daemon=True).start()

    def _on_coords(self, msg: Float32MultiArray):
        if len(msg.data) == 6:
            with self._lock:
                self.coords_xyzrpy = [float(v) for v in msg.data]
            # 디버그 출력(선택)
            # self.get_logger().info(f"coords: {self.coords_xyzrpy}")
        else:
            self.get_logger().warn(f"/mycobot/coords len={len(msg.data)} (expect 6)")

    def _on_angles_deg(self, msg: Float32MultiArray):
        if len(msg.data) == 6:
            with self._lock:
                self.angles_deg = [float(v) for v in msg.data]
            # self.get_logger().info(f"angles_deg: {self.angles_deg}")
        else:
            self.get_logger().warn(f"/mycobot/angles_deg len={len(msg.data)} (expect 6)")


    # =========================================================
    # 내부 유틸
    def _wait_first_joint_states(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                ok = self.current_angles_rad is not None
            if ok:
                return True
            time.sleep(0.02)
        return False

    def _clamp_deg(self, arr_deg):
        return [max(-lim, min(lim, float(a))) for a, lim in zip(arr_deg, JOINT_LIMITS)]

    # =========================================================
    # 서비스 호출
    def _send_set_angles(self, angles_deg):
        try:
            req = SetAngles.Request()
            req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6 = self._clamp_deg(angles_deg)
            req.speed = int(self.speed)
            self.client_angles.call_async(req)
            self.get_logger().info(f'➡️ Move request(deg): {angles_deg} (speed={self.speed}%)')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ set_angles call failed: {e}')
            return False

    # =========================================================
    # 동기 이동
    def move_joint(self, target_deg):
        target_rad = [math.radians(v) for v in target_deg]
        with self._lock:
            self.target_angles_rad = target_rad[:]
        self.reached_event.clear()

        if not self._send_set_angles(target_deg):
            return
        # 🟩 추가: 초기 반응 지연 시간 보정 (로봇이 움직이기 시작할 때까지)
        time.sleep(0.5)

        t0 = time.time()
        while time.time() - t0 < self.reach_timeout:
            if self.reached_event.wait(timeout=self.check_period):
                self.get_logger().info('✅ Reached target (stable).')
                return
        self.get_logger().warn('⌛ Timeout waiting for target.')

    # =========================================================
    # 모니터 루프
    def _monitor_loop(self):
        ok_count = 0
        last_log = 0.0
        while self._run:
            time.sleep(self.check_period)
            with self._lock:
                cur = None if self.current_angles_rad is None else self.current_angles_rad[:]
                tgt = None if self.target_angles_rad is None else self.target_angles_rad[:]
                stamp = self.current_stamp
            if cur is None or tgt is None:
                ok_count = 0
                continue

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            st_sec = stamp.sec + stamp.nanosec * 1e-9
            stale = (now_sec - st_sec) > self.stale_after_sec

            diffs = [abs(c - t) for c, t in zip(cur, tgt)]
            max_err = max(diffs)

            if stale:
                ok_count = 0
                continue

            if max_err <= self.reach_tolerance_rad:
                ok_count += 1
                if ok_count >= self.consecutive_ok_needed:
                    self.reached_event.set()
            else:
                ok_count = 0

    def _exec_sequence(self, seq):
        for i, pose in enumerate(seq, 1):
            self.get_logger().info(f'➡️ Step {i}/{len(seq)}: {pose}')
            self.move_joint(pose)
        self.get_logger().info('✅ Sequence done.')

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
        execu = MultiThreadedExecutor(num_threads=1)  # 🔧 최소 2스레드
        execu.add_node(node)
        execu.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('🧩 Shutting down...')
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
