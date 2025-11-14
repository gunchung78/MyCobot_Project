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
from mycobot_interfaces.srv import SetAngles, SetCoords, GetCoords, GripperStatus, GetAngles, PumpStatus
import pymycobot
from packaging import version
from std_msgs.msg import Float32MultiArray

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
print('pymycobot library version meets the requirements!')
from pymycobot import MyCobot320


def acquire(lock_file):
    """Acquire a file lock to prevent concurrent access.

    Args:
        lock_file (str): Path to the lock file.

    Returns:
        int | None: File descriptor if lock acquired, None if failed.
    """
    try:
        file_descriptor = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    except OSError as erro_info:
        print(f"Failed to open lock file {lock_file}: {erro_info}")
        return None
    try:
        fcntl.flock(file_descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return file_descriptor
    except:
        os.close(file_descriptor)
        return None


def release(fd):
    """Release a previously acquired file lock.

    Args:
        fd (int): File descriptor of the lock file.
    """
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except:
        pass


class MyCobotDriver(Node):
    """ROS2 node for controlling the MyCobot320 robot arm.

    Provides publishers for joint states and services for controlling
    joints, coordinates, grippers, and pumps.
    """

    def __init__(self):
        """Initialize MyCobotDriver node, publishers, and services."""
        super().__init__('mycobot_driver_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.mc = MyCobot320(port, str(baud))
        if self.mc.get_fresh_mode() != 1:
            self.mc.set_fresh_mode(1)

        # 🟩 추가: 마지막 유효 관절각 캐싱용 변수
        self._last_valid_angles = None

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_angles_f32 = self.create_publisher(Float32MultiArray, '/mycobot/angles', 10)
        self.pub_coords_f32 = self.create_publisher(Float32MultiArray, '/mycobot/coords', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        # Service servers
        self.srv_angles = self.create_service(SetAngles, 'set_angles', self.set_angles_callback)
        self.srv_coords = self.create_service(SetCoords, 'set_coords', self.set_coords_callback)
        self.srv_get_coords = self.create_service(GetCoords, 'get_coords', self.get_coords_callback)
        self.srv_get_angles = self.create_service(GetAngles, 'get_angles', self.get_angles_callback)
        self.srv_gripper = self.create_service(GripperStatus, 'set_gripper', self.set_gripper_callback)
        self.srv_pump = self.create_service(PumpStatus, 'set_pump_status', self.set_pump_callback)
        self.srv_force_gripper = self.create_service(GripperStatus, 'set_force_gripper', self.set_force_gripper_callback)

    def publish_joint_states(self):
        """Publish joint_states + (angles/coords) at 50 Hz."""
        try:
            lock = acquire('/tmp/mycobot_lock')
            if lock is None:
                self.get_logger().warn("publish_joint_states: lock busy, skip once")
                return

            # ⬇️ 한 번의 락에서 두 값 모두 읽기
            angles = self.mc.get_angles()          # e.g. [deg, ...] len==6
            coords = self.mc.get_coords()          # e.g. [x,y,z,rx,ry,rz]
            release(lock)

            # ---- 각도 캐시/유효성 ----
            if not isinstance(angles, list) or len(angles) != 6:
                if self._last_valid_angles is None:
                    return
                angles = self._last_valid_angles
            else:
                self._last_valid_angles = angles[:]

            # ---- joint_states (rad) 기존 로직 유지 ----
            js = JointState()
            js.header = Header()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6",
            ]
            js.position = [math.radians(a) for a in angles]  # rad로 발행(표준)
            self.pub.publish(js)

            # ---- 추가1: angles_deg 토픽 (그대로 deg) ----
            msg_ang = Float32MultiArray()
            msg_ang.data = [float(a) for a in angles]
            self.pub_angles_f32.publish(msg_ang)

            # ---- 추가2: coords 토픽 ----
            if isinstance(coords, list) and len(coords) == 6:
                msg_xyzrpy = Float32MultiArray()
                # 필요 시 mm→m, deg→rad 변환 가능하나, 요청대로 원형 그대로 발행
                msg_xyzrpy.data = [float(c) for c in coords]  # [x,y,z,rx,ry,rz]
                self.pub_coords_f32.publish(msg_xyzrpy)
            else:
                # 좌표가 일시적으로 비정상(-1 포함 등)일 수 있어 경고만
                self.get_logger().warn(f"coords invalid or len!=6: {coords}")

        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Joint state publish error: {e}")


    def set_angles_callback(self, request, response):
        """Set joint angles via ROS2 service.

        Args:
            request (SetAngles.Request): The request object containing target joint
                angles (joint_1 to joint_6) and movement speed.
            response (SetAngles.Response): The response object that will be updated
                with the operation result.

        Returns:
            SetAngles.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        lock = acquire('/tmp/mycobot_lock')
        if lock is None:
            self.get_logger().warn("set_angles: lock busy, skip")
            response.flag = False
            return response
        try:
            angles = [
                request.joint_1, request.joint_2, request.joint_3,
                request.joint_4, request.joint_5, request.joint_6,
            ]
            self.mc.send_angles(angles, request.speed)
            time.sleep(0.05)
            cur = self.mc.get_angles()
            if cur and len(cur) == 6:
                self._last_valid_angles = cur[:]  # 🟩 즉시 캐시 갱신
            response.flag = True
        except Exception:
            self.get_logger().error("SetJointAngles service error:\n" + traceback.format_exc())
            response.flag = False
        finally:
            release(lock)
        return response

    def set_coords_callback(self, request, response):
        """Set end-effector coordinates via ROS2 service.

        Args:
            request (SetCoords.Request): The request object containing target coordinates
                (x, y, z, rx, ry, rz), motion speed, and movement model.
            response (SetCoords.Response): The response object that will be updated
                with the operation result.

        Returns:
            SetCoords.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        try:
            lock = acquire('/tmp/mycobot_lock')
            coords = [request.x, request.y, request.z, request.rx, request.ry, request.rz]
            self.mc.send_coords(coords, request.speed, request.model)
            release(lock)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Set coords failed: {e}")
            response.flag = False
        return response

    def get_coords_callback(self, request, response):
        """Get current end-effector coordinates.

        Args:
            request (GetCoords.Request): The request object (unused in this service).
            response (GetCoords.Response): The response object that will be updated
                with the current robot end-effector coordinates.

        Returns:
            GetCoords.Response: The response containing the current coordinates
            (x, y, z, rx, ry, rz).
        """
        try:
            lock = acquire('/tmp/mycobot_lock')
            coords = self.mc.get_coords()
            release(lock)
            if not coords or len(coords) != 6:
                return
            if coords and all(c != -1 for c in coords) and len(coords) == 6:
                response.x, response.y, response.z, response.rx, response.ry, response.rz = coords
            else:
                self.get_logger().error("Failed to get coordinates.")
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"GetCoords service error: {e}")
        return response

    def get_angles_callback(self, request, response):
        """Get current joint angles.

        Args:
            request (GetAngles.Request): The request object (unused in this service).
            response (GetAngles.Response): The response object that will be updated
                with the current joint angles.

        Returns:
            GetAngles.Response: The response containing six joint angles.
        """
        lock = acquire('/tmp/mycobot_lock')
        if lock is None:
            self.get_logger().warn("get_angles: lock busy, skip")
            return response
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6 and all(a != -1 for a in angles):
                (response.joint_1, response.joint_2, response.joint_3,
                 response.joint_4, response.joint_5, response.joint_6) = angles
            else:
                self.get_logger().warn("Invalid get_angles response (None or wrong length).")
        except Exception:
            self.get_logger().error("GetAngles service error:\n" + traceback.format_exc())
        finally:
            release(lock)
        return response

    def set_gripper_callback(self, request, response):
        """Control the standard gripper open/close state.

        Args:
            request (GripperStatus.Request): The request object containing the gripper
                status (`True` = open, `False` = close).
            response (GripperStatus.Response): The response object that will be updated
                with the operation result.

        Returns:
            GripperStatus.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        try:
            lock = acquire('/tmp/mycobot_lock')
            speed = 80
            if request.status:
                self.mc.set_gripper_state(0, speed, 1)  # open gripper
            else:
                self.mc.set_gripper_state(1, speed, 1)  # close gripper
            release(lock)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetGripper service error: {e}")
            response.flag = False
        return response

    def set_force_gripper_callback(self, request, response):
        """Control the force gripper open/close state.

        Args:
            request (GripperStatus.Request): The request object containing the force
                gripper status (`True` = open, `False` = close).
            response (GripperStatus.Response): The response object that will be updated
                with the operation result.

        Returns:
            GripperStatus.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        try:
            lock = acquire('/tmp/mycobot_lock')
            if request.status:
                self.mc.set_pro_gripper_open()
            else:
                self.mc.set_pro_gripper_close()
            release(lock)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetForceGripper service error: {e}")
            response.flag = False
        return response

    def set_pump_callback(self, request, response):
        """Control the suction pump status.

        Args:
            request (PumpStatus.Request): The request object containing pump status
                (`True` = enable suction, `False` = release) and GPIO pin parameters.
            response (PumpStatus.Response): The response object that will be updated
                with the operation result.

        Returns:
            PumpStatus.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        try:
            lock = acquire('/tmp/mycobot_lock')
            pin1, pin2 = request.pin1, request.pin2
            if request.status:
                self.mc.set_basic_output(pin1, 0)
                time.sleep(0.05)
            else:
                self.mc.set_basic_output(pin1, 1)
                time.sleep(0.05)
            release(lock)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetPump service error: {e}")
            response.flag = False
        return response


def main(args=None):
    """Main entry point for running the MyCobotDriver node."""
    rclpy.init(args=args)
    node = MyCobotDriver()
    try:
        execu = MultiThreadedExecutor(num_threads=2)  # 🔧
        execu.add_node(node)
        execu.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
