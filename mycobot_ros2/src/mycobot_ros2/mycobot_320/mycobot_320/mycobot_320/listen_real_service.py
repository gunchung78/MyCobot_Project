import math
import time
import os
import fcntl
import rclpy
import traceback
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rclpy.executors import MultiThreadedExecutor
from mycobot_interfaces.srv import SetAngles, SetCoords, GetCoords, GripperStatus, GetAngles
import pymycobot
from packaging import version
from pymycobot import MyCobot320

MIN_REQUIRE_VERSION = '3.6.1'

def _check_version():
    cur = pymycobot.__version__
    print(f'current pymycobot library version: {cur}')
    if version.parse(cur) < version.parse(MIN_REQUIRE_VERSION):
        raise RuntimeError(
            f'pymycobot>={MIN_REQUIRE_VERSION} required, current={cur}. Please upgrade.'
        )
    print('pymycobot library version meets the requirements!')

LOCK_FILE = '/tmp/mycobot_lock'

def acquire(lock_file: str):
    try:
        fd = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    except OSError as e:
        print(f"Failed to open lock file {lock_file}: {e}")
        return None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fd
    except Exception:
        os.close(fd)
        return None

def release(fd: int):
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except Exception:
        pass

class MyCobotDriver(Node):
    """
    ROS2 node for controlling the MyCobot320 robot arm.
    - Publishes /joint_states at 50 Hz (rad)
    - Services:
      * /set_angles(SetAngles) : degrees + speed
      * /set_coords(SetCoords) : [x,y,z,rx,ry,rz] in mm/deg + speed + model
      * /get_angles(GetAngles)
      * /get_coords(GetCoords)
      * /set_gripper(GripperStatus) : standard gripper open/close
    """

    def __init__(self):
        super().__init__('mycobot_driver_node')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('default_move_speed', 30)  # 0~100
        self.declare_parameter('default_model', 0)        # 0:moveJ, 1:moveL, 2:moveC (라이브러리 기준)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.default_speed = int(self.get_parameter('default_move_speed').get_parameter_value().integer_value)
        self.default_model = int(self.get_parameter('default_model').get_parameter_value().integer_value)

        # --- Device init ---
        self.mc = MyCobot320(port, baud)  # baud를 정수로 사용
        try:
            self.mc.set_fresh_mode(0)
            self.mc.set_gripper_mode(0)
        except Exception:
            self.get_logger().warn("fresh_mode check/set not supported on this firmware, continue.")

        self._last_valid_angles = None

        # --- Publisher & Timer ---
        # self.pub = self.create_publisher(JointState, 'joint_states', 10)
        # self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz

        # --- Services ---
        self.create_service(SetAngles, 'set_angles', self.set_angles_callback)
        self.create_service(SetCoords, 'set_coords', self.set_coords_callback)
        self.create_service(GetCoords, 'get_coords', self.get_coords_callback)
        self.create_service(GetAngles, 'get_angles', self.get_angles_callback)
        self.create_service(GripperStatus, 'set_gripper', self.set_gripper_callback)

    # ---------- Helpers ----------
    def _with_lock(self):
        fd = acquire(LOCK_FILE)
        if fd is None:
            self.get_logger().warn("lock busy, skip once")
        return fd

    # ---------- Publisher ----------
    def publish_joint_states(self):
        try:
            fd = self._with_lock()
            if fd is None:
                return
            angles = self.mc.get_angles()
            release(fd)

            if not isinstance(angles, list) or len(angles) != 6:
                if self._last_valid_angles is None:
                    return
                angles = self._last_valid_angles
            else:
                self._last_valid_angles = angles[:]

            js = JointState()
            js.header = Header()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6"
            ]
            js.position = [math.radians(a) for a in angles]
            self.pub.publish(js)
        except Exception:
            self.get_logger().error("Joint state publish error:\n" + traceback.format_exc())

    # ---------- Services ----------
    def set_angles_callback(self, req, res):
        fd = self._with_lock()
        if fd is None:
            res.flag = False
            return res
        try:
            angles = [req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6]
            speed = int(req.speed) if req.speed > 0 else self.default_speed
            self.mc.sync_send_angles(angles, speed, 10)
            time.sleep(0.05)
            cur = self.mc.get_angles()
            if isinstance(cur, list) and len(cur) == 6:
                self._last_valid_angles = cur[:]
            res.flag = True
        except Exception:
            self.get_logger().error("SetAngles error:\n" + traceback.format_exc())
            res.flag = False
        finally:
            release(fd)
        return res

    def set_coords_callback(self, req, res):
        fd = self._with_lock()
        if fd is None:
            res.flag = False
            return res
        try:
            coords = [req.x, req.y, req.z, req.rx, req.ry, req.rz]  # mm/deg
            speed = int(req.speed) if req.speed > 0 else self.default_speed
            model = int(req.model) if req.model in (0, 1, 2) else self.default_model
            self.mc.sync_send_coords(coords, speed, model, 10)
            res.flag = True
        except Exception:
            self.get_logger().error("SetCoords error:\n" + traceback.format_exc())
            res.flag = False
        finally:
            release(fd)
        return res

    def get_coords_callback(self, req, res):
        fd = self._with_lock()
        if fd is None:
            return res
        try:
            coords = self.mc.get_coords()
            if isinstance(coords, list) and len(coords) == 6 and all(c != -1 for c in coords):
                res.x, res.y, res.z, res.rx, res.ry, res.rz = coords
            else:
                self.get_logger().warn("get_coords returned invalid data")
        except Exception:
            self.get_logger().error("GetCoords error:\n" + traceback.format_exc())
        finally:
            release(fd)
        return res

    def get_angles_callback(self, req, res):
        fd = self._with_lock()
        if fd is None:
            return res
        try:
            ang = self.mc.get_angles()
            if isinstance(ang, list) and len(ang) == 6 and all(a != -1 for a in ang):
                (res.joint_1, res.joint_2, res.joint_3,
                 res.joint_4, res.joint_5, res.joint_6) = ang
            else:
                self.get_logger().warn("get_angles returned invalid data")
        except Exception:
            self.get_logger().error("GetAngles error:\n" + traceback.format_exc())
        finally:
            release(fd)
        return res

    def set_gripper_callback(self, req, res):
        # 표준 그리퍼: set_gripper_state(0:open, 1:close, wait=1)
        fd = self._with_lock()
        if fd is None:
            res.flag = False
            return res
        try:
            speed = 80  # 고정
            if req.status:
                self.mc.set_gripper_value(100, speed, 1)  # open
            else:
                self.mc.set_gripper_value(10, speed, 1)  # close
            res.flag = True
        except Exception:
            self.get_logger().error("SetGripper error:\n" + traceback.format_exc())
            res.flag = False
        finally:
            release(fd)
        return res

def main(args=None):
    _check_version()
    rclpy.init(args=args)
    node = MyCobotDriver()
    try:
        exe = MultiThreadedExecutor(num_threads=2)
        exe.add_node(node)
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
