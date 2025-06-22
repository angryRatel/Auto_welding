# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_getting_position", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            set_tool,
            set_tcp,
            movej,
            movel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def move_and_log(move_type, position, index, vel=VELOCITY, acc=ACC):
        if move_type == "joint":
            print(f"Moving to joint position: {position}")
            movej(position, vel=vel, acc=acc)
        elif move_type == "task":
            print(f"Moving to task position: {position}")
            movel(position, vel=vel, acc=acc)
        else:
            raise ValueError("Invalid move type. Use 'joint' or 'task'.")

        pos, _ = get_current_posx()
        print(f"current position{index} : {pos}")

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0, 0, 90, 0, 90, 0])
    pos1 = posx([496.06, 93.46, 296.92, 20.75, 179.00, 19.09])
    pos2 = posx([548.70, -193.46, 96.92, 20.75, 179.00, 19.09])
    pos3 = posx([596.70, -7.46, 196.92, 20.75, 179.00, 19.09])

    if rclpy.ok():
        move_and_log("joint", JReady, 1)
        move_and_log("task", pos1, 2)
        move_and_log("task", pos2, 3)
        move_and_log("task", pos3, 4)
        move_and_log("joint", JReady, 5)
    rclpy.shutdown()


if __name__ == "__main__":
    main()