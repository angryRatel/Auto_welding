# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

import numpy as np

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_movesx_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            DR_BASE,
            movesx,
            DR_MVS_VEL_CONST,
            get_current_posx,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def generate_sine_path(points1, points2, step=0.1, amplitude_scale=1.0):
        """
        두 점 사이를 사인 곡선 형태로 연결하는 경로를 생성
        
        Parameters:
            points1 (list): 시작점 좌표
            points2 (list): 끝점 좌표 (x값만 다르고 나머지는 동일하다고 가정)
            step (float): 곡선 샘플링 간격
            amplitude_scale (float): 사인 곡선의 진폭 배율

        Returns:
            list: posx로 변환된 사인 곡선 경로 리스트
        """
        # x축 기준 거리 계산
        period = points2[0] - points1[0]
        scale = period / (2 * np.pi)

        # 사인곡선 샘플링
        x = np.arange(0, 2 * np.pi, step)
        x_world = x * scale + points1[0]
        y_world = np.sin(x) * scale * amplitude_scale + points1[1]

        # 고정된 나머지 좌표들
        common_coords = points1[2:]

        # 사인 곡선 좌표 생성
        sine_list = [posx([xi, yi, *common_coords]) for xi, yi in zip(x_world, y_world)]

        return sine_list

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0, 0, 90, 0, 90, 0])
    print(f"Moving to joint position: {JReady}")
    movej(JReady, vel=VELOCITY, acc=ACC)

    points1 = [277.145, 7.384, 34.081, 109.384, 179.97, 109.228]
    points2 = [707.81, 7.384, 34.081, 109.384, 179.97, 109.228]
    print(f"Setting to task position1: {points1}")
    print(f"Setting to task position2: {points2}")

    sine_list = generate_sine_path(points1, points2, step=0.1, amplitude_scale=1.0)
    print(f"Sine list length: {len(sine_list)} | Second value: {sine_list[1]}")

    print("Starting movesx: position1 → position2 using the Sine list")
    movesx(sine_list, vel=[100, 30], acc=[100, 60], ref=DR_BASE, vel_opt=DR_MVS_VEL_CONST)

    pos, _ = get_current_posx()
    print(f"current position : {pos}")

    print(f"Moving to joint position: {JReady}")
    movej(JReady, vel=VELOCITY, acc=ACC)
    rclpy.shutdown()


if __name__ == "__main__":
    main()