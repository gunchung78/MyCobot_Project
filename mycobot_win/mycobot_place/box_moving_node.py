# box_moving_node.py
import time

# ----------------------------------------
# 적재 동작 전용 모듈 (픽업 이후 단계)
# ----------------------------------------


def move_and_wait(mc, mode, value, speed=25, delay=0.5):
    """원래 코드와 동일"""
    if mode == "coords":
        mc.send_coords(value, speed, 0)
    elif mode == "angles":
        mc.send_angles(value, speed)
    else:
        raise ValueError("mode must be 'coords' or 'angles'")
    while mc.is_moving():
        time.sleep(0.1)
    time.sleep(delay)


def place_box(mc, val, idx=0):

    z = idx * 25;

    if val in ('green', 'normal'):

        z = z + 150
        
        move_and_wait(mc, "angles", [-10,0, 78.95, -21,-87.36,-15])
    
        move_and_wait(mc, "coords", [-293.5, -25, z, -176, 0, 90], 10, 1)

        mc.set_gripper_value(40,20,1)

        move_and_wait(mc, "angles", [-22.23,0,78.95,-21,-87.36,-15])

    elif(val == 'blue'):

        z = z + 170

        move_and_wait(mc, "angles", [12.12, 0 ,70.83,-16.08,-67.5, -150])
    
        move_and_wait(mc, "coords", [-219.2, -281.9, z, 179.41, -5, -100], 10, 1)
        
        mc.set_gripper_value(40,20,1)

        move_and_wait(mc, "angles", [12.12, 0 ,70.83,-16.08,-67.5, -150])

    elif val in ('red', 'anomaly'):

        z = z + 160

        move_and_wait(mc, "angles", [136.66, 0 ,-55.98, 0 , 109.51, -30])
    
        move_and_wait(mc, "coords", [-180, 240, z, -173.15, 0, 90], 10, 1)

        mc.set_gripper_value(40,20,1)

        move_and_wait(mc, "angles", [136.66, 0 ,-55.98, 0 ,109.51, -30])

    move_and_wait(mc, "angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])
    move_and_wait(mc, "angles", [5.09, 0, -80, -0, 90, -90])
