import tkinter as tk
from tkinter import filedialog, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib
import platform
import numpy as np
import cv2
import os
import threading
import csv
import sounddevice as sd  # 경고음용 추가
import time


# 시스템별 폰트 설정
if platform.system() == 'Windows':
   matplotlib.rc('font', family='Malgun Gothic')
elif platform.system() == 'Darwin':
   matplotlib.rc('font', family='AppleGothic')
else:
   matplotlib.rc('font', family='NanumGothic')
matplotlib.rcParams['axes.unicode_minus'] = False


weld_csv_output = os.path.join(os.path.expanduser('~'), 'weld_points.csv')


# 경고음 재생 함수
def play_warning_sound():
   fs = 44100
   duration = 0.5
   f = 1000.0
   samples = (np.sin(2 * np.pi * np.arange(fs * duration) * f / fs)).astype(np.float32)
   sd.play(samples, samplerate=fs)
   sd.wait()


# Z축 offset 적용 함수
def offset_z(pose, dz):
   return [pose[0], pose[1], pose[2] + dz, pose[3], pose[4], pose[5]]


# 팝업 경고창 표시 함수
def show_impact_popup(pos):
   popup = tk.Tk()
   popup.withdraw()
   msg = f" 충격이 감지되었습니다!\n위치: {pos}"
   messagebox.showwarning("충격 감지", msg)
   popup.destroy()


# 이미지 경로 불러오기
def load_image():
   file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp")])
   if file_path:
       entry_path.delete(0, tk.END)
       entry_path.insert(0, file_path)


# 이미지에서 포인트 추출
def extract_raw_points(image_path):
   img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
   if img is None:
       raise ValueError("이미지를 불러올 수 없습니다.")
   _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
   try:
       skeleton = cv2.ximgproc.thinning(binary)
   except AttributeError:
       raise ImportError("cv2.ximgproc 모듈이 필요합니다. 'opencv-contrib-python'을 설치하세요.")
   points = np.column_stack(np.where(skeleton > 0))
   points = points[:, [1, 0]]
   return img, points


# 포인트 스케일링
def scale_points_to_target(points):
   x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
   y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
   original_width = x_max - x_min
   original_height = y_max - y_min


   target_x_min, target_x_max = 280, 577
   target_y_min, target_y_max = -155, 220
   target_width = target_x_max - target_x_min
   target_height = target_y_max - target_y_min


   scale = min(target_width / original_width, target_height / original_height)
   scaled_width = original_width * scale
   scaled_height = original_height * scale
   offset_x = target_x_min + (target_width - scaled_width) / 2
   offset_y = target_y_min + (target_height - scaled_height) / 2


   scaled_points = np.empty_like(points, dtype=np.float32)
   scaled_points[:, 0] = (points[:, 0] - x_min) * scale + offset_x
   scaled_points[:, 1] = (points[:, 1] - y_min) * scale + offset_y
   return scaled_points


# 충격 감지 함수
def monitor_impact(get_tool_force, get_current_posx, movel, VELOCITY, ACC):
   threshold = 10.0
   force = get_tool_force()
   if abs(force[0]) > threshold or abs(force[1]) > threshold or abs(force[2]) > threshold:
       impact_pos = get_current_posx()
       print(f" 충격 감지됨! 위치: {impact_pos}")
       show_impact_popup(impact_pos)
       play_warning_sound()
       lifted_pos = offset_z(impact_pos, 50)
       try:
           movel(lifted_pos, VELOCITY, ACC, mod=0)
       except TypeError:
           movel(lifted_pos, VELOCITY, ACC)
       return True
   return False


# 용접 포인트 추출 및 시각화
def start_welding():
   image_path = entry_path.get()
   if not os.path.exists(image_path):
       messagebox.showerror("오류", "이미지 경로가 유효하지 않습니다.")
       return


   try:
       img, points = extract_raw_points(image_path)
   except Exception as e:
       messagebox.showerror("오류", str(e))
       return


   raw_csv = os.path.join(os.path.dirname(image_path), 'weld_points_raw.csv')
   with open(raw_csv, 'w', newline='', encoding='utf-8-sig') as f:
       writer = csv.writer(f)
       writer.writerow(['x', 'y'])
       for p in points:
           writer.writerow(p)


   messagebox.showinfo("완료", f"원본 좌표 CSV 파일이 저장되었습니다:\n{raw_csv}")


   scaled_points = scale_points_to_target(points)
   points_sorted = scaled_points[np.argsort(scaled_points[:, 0])]
   step = 10
   points_sampled = points_sorted if len(points_sorted) < step else points_sorted[::step]


   with open(weld_csv_output, 'w', newline='', encoding='utf-8-sig') as f:
       writer = csv.writer(f)
       writer.writerow(['x', 'y'])
       for p in points_sampled:
           writer.writerow(p)


   ax.clear()
   ax.scatter(points_sampled[:, 0], points_sampled[:, 1], c='red', s=10, label='weld Points')
   ax.set_title("용접 경로 예상 화면")
   ax.set_xlabel("X 좌표")
   ax.set_ylabel("Y 좌표")
   ax.set_xlim(280, 577)
   ax.set_ylim(-155, 220)
   ax.set_aspect('equal')
   ax.legend()
   ax.grid(True)
   canvas.draw()


# 로봇 동작 함수
def execute_robot():
   import rclpy
   import DR_init
   from time import sleep


   # 로봇 ID 및 모델
   ROBOT_ID = "dsr01"
   ROBOT_MODEL = "m0609"
   VELOCITY, ACC = 60, 60


   # ROS2 및 노드 초기화
   rclpy.init()
   node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)


   # DSR 전역 초기화
   DR_init.__dsr__id = ROBOT_ID
   DR_init.__dsr__model = ROBOT_MODEL
   DR_init.__dsr__node = node


   # g_node 직접 설정 (dsr_init 없이)
   import DSR_ROBOT2
   DSR_ROBOT2.g_node = node


   # DSR API 가져오기
   from DSR_ROBOT2 import set_tool, set_tcp, movej, movel, get_current_posx, get_tool_force, task_compliance_ctrl, release_compliance_ctrl
   from DR_common2 import posx, posj


   # 툴 및 TCP 설정
   set_tool("Tool Weight_2FG")
   set_tcp("2FG_TCP")


   # CSV 경로 확인
   if not os.path.exists(weld_csv_output):
       print("CSV 파일을 찾을 수 없습니다.")
       return


   # 좌표 로딩
   points = []
   with open(weld_csv_output, newline='', encoding='utf-8-sig') as csvfile:
       reader = csv.reader(csvfile)
       next(reader)
       for row in reader:
           x, y = float(row[0]), float(row[1])
           points.append((x, y))


   # 초기 위치로 이동
   movej([0, 0, 90, 0, 90, 0], 30, 30)


   #time.sleep(1)


   #task_compliance_ctrl(stx=[10, 10, 10, 10, 10, 10])


   #time.sleep(1)


   i = 0
   n = len(points)


   while rclpy.ok() and i < n:
       x, y = points[i]
       position = [x, y, 102.0, 151.75, 179.00, 151.09]
       print(f"{i}: 이동 -> {position}")
       movel(position, VELOCITY, ACC)


       # 충격 감지 시 종료 처리
       if monitor_impact(get_tool_force, get_current_posx, movel, VELOCITY, ACC):
           root.quit()
           break


       i += 1


   #time.sleep(1)
   #release_compliance_ctrl()
   # 복귀
   movej([0, 0, 90, 0, 90, 0], 30, 30)
   rclpy.shutdown()
   root.destroy()




# 로봇 쓰레드 실행
def start_robot_thread():
   threading.Thread(target=execute_robot, daemon=True).start()


# GUI 설정
root = tk.Tk()
root.title("용접 경로 추출기")
root.geometry("900x700")


frame_top = tk.Frame(root)
frame_top.pack(pady=10)


entry_path = tk.Entry(frame_top, width=60)
entry_path.pack(side=tk.LEFT, padx=5)


btn_browse = tk.Button(frame_top, text="이미지 선택", command=load_image)
btn_browse.pack(side=tk.LEFT, padx=5)


frame_buttons = tk.Frame(root)
frame_buttons.pack(pady=10)


btn_extract = tk.Button(frame_buttons, text="용접 포인트 추출", command=start_welding, bg='green', fg='white', width=20, height=2)
btn_extract.pack(side=tk.LEFT, padx=5)


btn_weld_start = tk.Button(frame_buttons, text="용접 시작", command=start_robot_thread, width=20, height=2)
btn_weld_start.pack(side=tk.LEFT, padx=5)


fig, ax = plt.subplots(figsize=(8, 6))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()


root.mainloop()







