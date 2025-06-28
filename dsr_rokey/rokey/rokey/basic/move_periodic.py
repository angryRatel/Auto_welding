# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30


import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move_periodic", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            amove_periodic,
            set_tool,
            set_tcp,
            movej,
            DR_TOOL,
        )

        from DR_common2 import posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_RG2")
    set_tcp("RG2_TCP")
    
    JReady = posj([0, 0, 90, 0, 90, 0])
    example_amp = [0.0, 0.0, 0.0, 0.0, 0.0, 30.0]

    if rclpy.ok():
        
        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Starting amove_periodic: {example_amp}")
        amove_periodic(amp=example_amp, period=1.0, atime=0.02, repeat=3, ref=DR_TOOL)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
