from pymycobot import MyCobot320
import time
mc = MyCobot320("/dev/ttyACM0", 115200)

print("adaptive open/close")
mc.set_gripper_value(60, 80, 1); time.sleep(2)
mc.set_gripper_value(20, 80, 1); time.sleep(2)

print("parallel open/close")
mc.set_gripper_state(0, 80, 3); time.sleep(2)
mc.set_gripper_state(1, 80, 3); time.sleep(2)
