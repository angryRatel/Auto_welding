# Brik Assemble @20250515

import rclpy
import DR_init
import time
import sys

# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
ON, OFF = 1, 0

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("brick_assemble_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node


############ Main ##############
def main(args=None):


    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_force,
            release_compliance_ctrl,
            get_digital_input,
            set_digital_output,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    ############ Grippper Grip & Rease ##############
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print("Wait for digital input", sig_num)
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    ##### Pose Define ####
    pos_home = posj(0, 0, 90, 0, 90, 0)
    block_1_grip = posx(322.63, -269.22, 55.76, 53.41, -179.57, 53.82)
    block_2_grip = posx(395.02, -272.56, 44.0, 2.06, -179.34, 2.1) 

    block_1_grip_down = posx(0, 0, -40, 0, 0, 0)
    block_1_grip_up = posx(0, 0, 100, 0, 0, 0)

    block_2_grip_down = posx(0, 0, -30, 0, 0, 0)
    block_2_grip_up = posx(0, 0, 100, 0, 0, 0)

    target1 = posx(379.44, -110.61, 45.07, 30.37, -179.54, 30.45) # 블럭1 놓을 위치의 위로 이동
    target2 = posx(387.24, -113.72, 55.11, 35.65, -179.46, 35.8) # 블럭2 놓을 위치의 위로 이동

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        if(release_force() == 0):
            print("release force")
        time.sleep(0.5)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")

        # 동작 전 그리퍼 열림 확인
        release()
        print("Gripper Release")

        # Home Pose
        movej(pos_home, vel = VELOCITY, acc = ACC)
        print("HomePose Moved")
        time.sleep(0.1)

        ########### Block 1 Assemble ############
        # Block_1 Grip
        
        movel(block_1_grip, vel = VELOCITY, acc = ACC) #블럭 위로 이동
        print("블럭 위로 이동")
        time.sleep(0.1)

        movel(block_1_grip_down, vel = VELOCITY, acc = ACC, mod=1) #블럭 위치로 이동
        print("블럭 그립 위치로 다운")
        time.sleep(0.1)
        grip() # 블럭 집기
        print("블럭 집기")
        time.sleep(0.1)
        movel(block_1_grip_up, vel = VELOCITY, acc = ACC, mod=1) #블럭 들어올리기
        print("블럭 들기")


        # Assemble_1
        movel(target1, vel = VELOCITY, acc = ACC)

        ret = task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        if ret == 0:
             print("Compliance_ctrl Set")
        else:
             print("Compliance_ctrl failed!!")

        time.sleep(0.1)
        ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        if ret == 0:
             print("set_desired_force Set")
        else:
             print("set_desired_force Set Failed!!!!!")

        time.sleep(0.1)

        force_condition = 0
        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=15)
            time.sleep(0.1)
        
        if(release_force() == 0):
            print("release force")
        else:
             print("release force Failed!!!!")
        time.sleep(0.1)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")
        else:
             print("release compliance ctrl Failed!!!!")

        release()

        time.sleep(0.1)

        target1_up = posx(0, 0, 100, 0, 0, 0)
        movel(target1_up, vel = VELOCITY, acc = ACC, mod=1)


        ########### Block 2 Assemble ############
        # Block_2 Grip
        movel(block_2_grip, vel = VELOCITY, acc = ACC)

        movel(block_2_grip_down, vel = VELOCITY, acc = ACC, mod=1)
        time.sleep(0.1)
        grip()
        time.sleep(0.1)
        movel(block_2_grip_up, vel = VELOCITY, acc = ACC, mod=1)

        # Assemble_2
        movel(target2, vel = VELOCITY, acc = ACC)

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)

        force_condition = 0
        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=15)
            time.sleep(0.1)

        
        if(release_force() == 0):
            print("release force")
        else:
             print("release force Failed!!!!")
        time.sleep(0.1)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")
        else:
             print("release compliance ctrl Failed!!!!")

        release()

        time.sleep(0.1)

        target2_up = posx(0, 0, 100, 0, 0, 0)
        movel(target2_up, vel = 30, acc = 30, mod=1)


    rclpy.shutdown()
if __name__ == "__main__":
    main()


