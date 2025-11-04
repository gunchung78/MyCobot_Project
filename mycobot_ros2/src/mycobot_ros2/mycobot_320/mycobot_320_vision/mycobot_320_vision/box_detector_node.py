#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

FOURCC_MAP = {
    'MJPG': 'MJPG',
    'YUYV': 'YUYV',   # (=YUY2)
    'H264': 'H264',
    '': None,
    None: None,
}

class BoxDetectorNode(Node):
    def __init__(self):
        super().__init__('box_detector_node')

        # ---------- Parameters ----------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('calc_roll', True)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fourcc', '')            # '', 'MJPG', 'YUYV', 'H264'
        self.declare_parameter('force_v4l2', True)      # V4L2 백엔드 우선
        self.declare_parameter('disable_gst', True)     # GStreamer 우선순위 낮추기
        self.declare_parameter('publish_overlay', True) # 오버레이 이미지 퍼블리시 여부

        # HSV ranges (list of 3 ints each)
        self.declare_parameter('lower_green', [35, 50, 50])
        self.declare_parameter('upper_green', [85, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 250])
        self.declare_parameter('upper_white', [180, 50, 255])

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.min_area     = int(self.get_parameter('min_area').value)
        self.calc_roll    = bool(self.get_parameter('calc_roll').value)
        self.frame_rate   = float(self.get_parameter('frame_rate').value)
        self.width        = int(self.get_parameter('width').value)
        self.height       = int(self.get_parameter('height').value)
        self.force_v4l2   = bool(self.get_parameter('force_v4l2').value)
        self.disable_gst  = bool(self.get_parameter('disable_gst').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self.fourcc_req   = FOURCC_MAP.get(self.get_parameter('fourcc').value, None)

        lg = np.array(self.get_parameter('lower_green').value, dtype=np.uint8)
        ug = np.array(self.get_parameter('upper_green').value, dtype=np.uint8)
        lw = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        uw = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.lower_green, self.upper_green = lg, ug
        self.lower_white, self.upper_white = lw, uw

        # GStreamer 우선순위 낮추기 (OpenCV가 먼저 V4L2를 고르도록)
        if self.disable_gst:
            os.environ['OPENCV_VIDEOIO_PRIORITY_GSTREAMER'] = '0'

        # ---------- Publishers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_uvr   = self.create_publisher(Point, '/box_detection/uvr', qos)
        self.pub_raw   = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_ovlay = self.create_publisher(Image, '/camera/overlay', 10)
        self.bridge    = CvBridge()

        # ---------- OpenCV Capture ----------
        self.cap = None
        self.fail_count = 0
        self._open_camera_or_throw()

        # ---------- Timer ----------
        period = 1.0 / max(self.frame_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        # 내부 상태
        self.last_u = None
        self.last_v = None
        self.last_roll = None
        self.last_area = 0

    # ------- camera open helpers -------
    def _try_open(self, index, backend=None, width=640, height=480, fps=30, fourcc=None):
        # 백엔드 선택
        if backend == 'v4l2':
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(index)

        # 지연 줄이기
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 해상도/프레임 설정
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          fps)

        # FOURCC 설정
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

        # 워밍업: 실제로 프레임이 들어오는지 테스트
        ok_reads = 0
        for _ in range(10):
            ret, f = cap.read()
            if ret and f is not None:
                ok_reads += 1
                break
            cv2.waitKey(10)

        if not cap.isOpened() or ok_reads == 0:
            cap.release()
            return None

        return cap

    def _open_camera_or_throw(self):
        idx = self.camera_index
        W, H, FPS = self.width, self.height, int(self.frame_rate)

        # 시나리오: 요청된 fourcc 우선 → MJPG → YUYV → auto
        fourcc_list = [self.fourcc_req, 'MJPG', 'YUYV', None]
        # 백엔드 우선순위
        backend_list = (['v4l2', None] if self.force_v4l2 else [None, 'v4l2'])

        self.cap = None
        for be in backend_list:
            for fc in fourcc_list:
                if fc is not None and fc not in FOURCC_MAP.values():
                    continue
                cap = self._try_open(idx, backend=be, width=W, height=H, fps=FPS, fourcc=fc)
                if cap:
                    self.cap = cap
                    self.get_logger().info(
                        f'카메라 오픈 성공: backend={be or "default"}, fourcc={fc or "auto"}, {W}x{H}@{FPS}'
                    )
                    return

        # 모두 실패
        raise RuntimeError(f'카메라 인덱스 {idx} 를 여는 데 실패했습니다. (백엔드/포맷 재시도 모두 실패)')

    def _reopen_camera(self):
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass
        self.cap = None
        self._open_camera_or_throw()
        self.fail_count = 0

    # ------------- main loop -------------
    def on_timer(self):
        ret, frame = (False, None)
        if self.cap:
            ret, frame = self.cap.read()

        if not ret or frame is None:
            self.fail_count += 1
            self.get_logger().info(f'프레임 읽기 실패…({self.fail_count}) 해상도/포맷/백엔드 재시도 중')
            # 연속 실패 누적되면 재오픈
            if self.fail_count >= 5:
                self.get_logger().warn('연속 프레임 실패로 카메라 재오픈 시도')
                try:
                    self._reopen_camera()
                except Exception as e:
                    self.get_logger().error(f'카메라 재오픈 실패: {e}')
            return

        # 성공적으로 프레임 받았으면 실패 카운터 리셋
        self.fail_count = 0

        # ---- 퍼블리시: 원본 이미지 ----
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub_raw.publish(raw_msg)
        except Exception as e:
            self.get_logger().warn(f'raw image publish 실패: {e}')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)
        final_mask = cv2.bitwise_or(mask_green, mask_white)

        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        u_c = v_c = None
        roll_angle = None
        area = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > self.min_area:
                if self.calc_roll:
                    rect = cv2.minAreaRect(largest)  # ((cx,cy), (w,h), angle)
                    (cx, cy), (w, h), angle = rect
                    u_c, v_c = int(cx), int(cy)
                    roll_angle = angle + 90.0 if w < h else angle  # 도(deg)
                else:
                    x, y, w, h = cv2.boundingRect(largest)
                    u_c, v_c = int(x + w / 2), int(y + h / 2)
                    roll_angle = 0.0  # 사용 안 함

        # 퍼블리시: 값이 없으면 음수로 표기(미검출)
        msg = Point()
        msg.x = float(u_c) if u_c is not None else -1.0
        msg.y = float(v_c) if v_c is not None else -1.0
        msg.z = float(roll_angle) if roll_angle is not None else -999.0
        self.pub_uvr.publish(msg)

        # ---- 퍼블리시: 오버레이 이미지 (옵션) ----
        if self.publish_overlay:
            try:
                vis = frame.copy()
                if u_c is not None and v_c is not None:
                    cv2.circle(vis, (int(u_c), int(v_c)), 6, (0, 0, 255), -1)
                    cv2.putText(
                        vis, f"u={int(u_c)} v={int(v_c)} roll={roll_angle if roll_angle is not None else 0:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
                    )
                ov_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                self.pub_ovlay.publish(ov_msg)
            except Exception as e:
                self.get_logger().warn(f'overlay image publish 실패: {e}')

        # 디버그 로그
        self.last_u, self.last_v, self.last_roll, self.last_area = msg.x, msg.y, msg.z, area
        self.get_logger().debug(
            f'pub u={msg.x:.1f}, v={msg.y:.1f}, roll={msg.z:.2f}°, area={int(area)}'
        )

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BoxDetectorNode()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f'노드 오류: {e}')
        else:
            print(f'노드 초기화 오류: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
