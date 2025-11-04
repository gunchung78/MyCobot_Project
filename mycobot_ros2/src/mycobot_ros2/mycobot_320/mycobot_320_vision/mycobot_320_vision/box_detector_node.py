#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BoxDetectorNode(Node):
    def __init__(self):
        super().__init__('box_detector_node')

        # ---------- Parameters ----------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('calc_roll', True)
        # HSV ranges (list of 3 ints each)
        self.declare_parameter('lower_green', [35, 50, 50])
        self.declare_parameter('upper_green', [85, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 250])
        self.declare_parameter('upper_white', [180, 50, 255])
        self.declare_parameter('frame_rate', 30.0)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.calc_roll = bool(self.get_parameter('calc_roll').value)
        lg = np.array(self.get_parameter('lower_green').value, dtype=np.uint8)
        ug = np.array(self.get_parameter('upper_green').value, dtype=np.uint8)
        lw = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        uw = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.lower_green, self.upper_green = lg, ug
        self.lower_white, self.upper_white = lw, uw
        self.frame_rate = float(self.get_parameter('frame_rate').value)

        # ---------- Publisher (u, v, roll_deg) ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_uvr = self.create_publisher(Point, '/box_detection/uvr', qos)

        # ---------- OpenCV Capture ----------
        # (ROS2는 OS별 VideoCapture 백엔드 자동 선택)
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'카메라 인덱스 {self.camera_index} 를 열 수 없습니다.')
            raise RuntimeError('Failed to open camera')

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'카메라 인덱스 {self.camera_index} 오픈: {width}x{height}')

        # ---------- Timer ----------
        period = 1.0 / max(self.frame_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        # 내부 상태
        self.last_u = None
        self.last_v = None
        self.last_roll = None
        self.last_area = 0

    def on_timer(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임을 읽지 못했습니다.')
            return

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

        # 로깅(레이트 제한)
        self.last_u, self.last_v, self.last_roll, self.last_area = msg.x, msg.y, msg.z, area
        self.get_logger().debug(f'pub u={msg.x:.1f}, v={msg.y:.1f}, roll={msg.z:.2f}°, area={int(area)}')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
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
