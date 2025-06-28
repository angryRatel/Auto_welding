# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

import time
import numpy as np

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_amove_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            amovel,
            DR_MV_RA_OVERRIDE,
            movel,
            get_current_posx,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 초기 위치
    JReady = posj([0, 0, 90, 0, 90, 0])
    print(f"Moving to joint position: {JReady}")
    movej(JReady, vel=VELOCITY, acc=ACC)

    point1 = posx([674.863, -10.427, 65.98, 101.889, -175.666, 109.147])
    point2 = posx([115.443, -510.482, 40.439, 24.334, -173.111, 102.941])
    point3 = posx([314.819, 374.565, 178.373, 51.638, 173.566, 10.385])
    print(f"Setting to task position1: {point1}, idx : {0}")
    print(f"Setting to task position2: {point2}, idx : {1}")
    print(f"Setting to task position3: {point3}, idx : {2}")

    points = [point1, point2, point3]
    movel(point1, vel=VELOCITY, acc=ACC)
    idx = 0

    while rclpy.ok():
        print(f"Moving asynchronously, idx: {idx}")
        pos, _ = get_current_posx()
        time.sleep(0.1)
        print(f"current position : {pos}")
        amovel(points[idx], vel=VELOCITY, acc=ACC, ra=DR_MV_RA_OVERRIDE)
        time.sleep(3)
        idx+=1
        idx %= 3

    rclpy.shutdown()

if __name__ == "__main__":
    main()
