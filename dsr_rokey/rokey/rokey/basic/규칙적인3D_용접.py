# a_home // movec 자리에 home 기능 넣음.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dsr_msgs2.srv import SetRobotMode




import tkinter as tk
from tkinter import StringVar, filedialog, messagebox
import threading
import csv
import numpy as np
import os


import DR_init
import sys
from time import sleep
import csv


# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ROS2 초기화
rclpy.init()
node = rclpy.create_node("welding_move", namespace=ROBOT_ID)
DR_init.__dsr__node = node


from DSR_ROBOT2 import set_tool, set_tcp, movej, movel, movec, mwait
from DR_common2 import posx, posj


# --- 유틸리티 함수들 (변경 없음) ---
def copy_to_clipboard(root_widget, string_var_to_copy):
  text_content = string_var_to_copy.get().strip()
  if text_content and text_content not in ["수신 대기중...", "캡처 대기중...", "계산 대기중...", "N/A", "계산 실패"]:
      root_widget.clipboard_clear()
      root_widget.clipboard_append(text_content)
      root_widget.update()
      print(f"클립보드에 복사됨: {text_content}")
  else:
      print("복사할 유효한 내용이 없습니다.")




def create_display_entry(parent_widget, default_display_text, grid_row, grid_col, entry_width=50, columnspan=1, readonly=False):
  entry_string_var = StringVar()
  entry_string_var.set(str(default_display_text))
  state_val = 'readonly' if readonly else 'normal'
  entry_widget = tk.Entry(parent_widget, textvariable=entry_string_var, width=entry_width, state=state_val)
  if readonly:
      entry_widget.config(relief=tk.FLAT, fg='black')
  entry_widget.grid(row=grid_row, column=grid_col, padx=5, pady=2, sticky="ew", columnspan=columnspan)
  return entry_string_var




# === 원의 중심 및 반지름 계산 함수 (GUI용) ===
# 변경점: 함수 이름 변경, 반지름 계산 및 반환 추가
def calculate_circle_center_and_radius_for_gui(p1_xyz, p2_xyz, p3_xyz, status_update_func=print): # 이름 변경
  P1 = np.array(p1_xyz, dtype=float)
  P2 = np.array(p2_xyz, dtype=float)
  P3 = np.array(p3_xyz, dtype=float)




  if np.allclose(P1, P2) or np.allclose(P1, P3) or np.allclose(P2, P3):
      status_update_func("오류: 두 개 이상의 점이 거의 동일합니다. 중심/반지름 계산 불가.")
      return None, None # << 변경: 반지름에 대해서도 None 반환




  v_ab = P2 - P1
  v_bc = P3 - P2
  normal_plane = np.cross(v_ab, v_bc)




  if np.linalg.norm(normal_plane) < 1e-7:
      status_update_func("오류: 세 점이 거의 한 직선 위에 있어 중심/반지름 정의 불가.")
      return None, None # << 변경: 반지름에 대해서도 None 반환




  M1 = (P1 + P2) / 2.0
  M2 = (P2 + P3) / 2.0
  n1 = np.cross(normal_plane, v_ab)
  n2 = np.cross(normal_plane, v_bc)




  if np.linalg.norm(n1) < 1e-7 or np.linalg.norm(n2) < 1e-7:
      status_update_func("오류: 수직이등분선 방향 벡터 계산 실패.")
      return None, None # << 변경: 반지름에 대해서도 None 반환
    
  A_mat = np.column_stack((n1, -n2))
  b_vec = M2 - M1




  try:
      params, _, _, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
      t = params[0]
      center = M1 + t * n1
    
      # >>> 추가된 부분: 반지름 계산 <<<
      radius = np.linalg.norm(center - P1)
    
      r2 = np.linalg.norm(center - P2)
      r3 = np.linalg.norm(center - P3)
      if not (np.isclose(radius, r2, atol=1e-3) and np.isclose(radius, r3, atol=1e-3)):
          status_update_func(
              f"경고: 중심-각 점 거리 (반지름) 약간 불일치. R1:{radius:.4f}, R2:{r2:.4f}, R3:{r3:.4f}"
          )
      return center.tolist(), radius # << 변경: 중심과 함께 반지름 반환
  except np.linalg.LinAlgError as e:
      status_update_func(f"선형대수 오류: {e}. 점들이 한 직선 위일 가능성.")
      return None, None # << 변경: 반지름에 대해서도 None 반환
  except Exception as e_gen:
      status_update_func(f"중심/반지름 계산 중 일반 오류: {e_gen}")
      return None, None # << 변경: 반지름에 대해서도 None 반환




# === ROS2 노드들 (변경 없음) ===
class ServiceClientNode(Node):
  def __init__(self, node_name="service_client_node_gui"):
      super().__init__(node_name)
      self.robot_mode_client = self.create_client(SetRobotMode, "/dsr01/system/set_robot_mode")




  def send_robot_mode_request(self, mode=0):
      if not self.robot_mode_client.service_is_ready():
          self.get_logger().info("로봇 모드 설정 서비스가 준비되지 않았습니다. 2초간 대기...")
          if not self.robot_mode_client.wait_for_service(timeout_sec=2.0):
              self.get_logger().error("로봇 모드 설정 서비스 연결 실패.")
              return None
      request = SetRobotMode.Request()
      request.robot_mode = mode
      future = self.robot_mode_client.call_async(request)
      return future




class PosTopicSubscriber(Node):
  def __init__(self, live_posx_var, live_posj_var):
      super().__init__("pos_topic_subscriber_gui")
      self.live_posx_var = live_posx_var
      self.live_posj_var = live_posj_var
      self.create_subscription(
          Float64MultiArray, "/dsr01/msg/current_posx", self.current_posx_callback, 10
      )
      self.create_subscription(
          Float64MultiArray, "/dsr01/msg/joint_state", self.joint_state_callback, 10
      )
      self.get_logger().info("좌표 토픽 구독자 시작됨.")




  def current_posx_callback(self, msg):
      try:
          data = [round(d, 3) for d in msg.data]
          self.live_posx_var.set(f"posx({data})")
      except Exception as e:
          self.get_logger().error(f"current_posx_callback 오류: {e}")




  def joint_state_callback(self, msg):
      try:
          data = [round(d, 3) for d in msg.data]
          self.live_posj_var.set(f"posj({data})")
      except Exception as e:
          self.get_logger().error(f"joint_state_callback 오류: {e}")




# === 메인 GUI 애플리케이션 클래스 ===
class RobotPointCaptureApp:
  def __init__(self, root_widget, service_client_node_instance):
      self.root = root_widget
      # 변경점: 창 제목에 버전 업데이트
      self.root.title("로봇 3점 캡처 및 원 중심/반지름 계산기 v3")
      self.service_client_node = service_client_node_instance
      self.captured_points_posx = [None, None, None]
      self.point_labels_text = ["P1 (끝점)", "P2 (중간점)", "P3 (시작점)"]
      self.captured_point_display_vars = []




      current_row = 0
      tk.Label(self.root, text="실시간 로봇 좌표:", font=('Arial', 10, 'bold')).grid(row=current_row, column=0, columnspan=3, sticky="w", padx=5, pady=5)
    
      current_row += 1
      tk.Label(self.root, text="TCP (posx):").grid(row=current_row, column=0, sticky="w", padx=10, pady=2)
      self.live_posx_display_var = create_display_entry(self.root, "수신 대기중...", current_row, 1, entry_width=60, readonly=True)
      tk.Button(self.root, text="복사", command=lambda: copy_to_clipboard(self.root, self.live_posx_display_var)).grid(row=current_row, column=2, padx=5, pady=2)




      current_row += 1
      tk.Label(self.root, text="관절 (posj):").grid(row=current_row, column=0, sticky="w", padx=10, pady=2)
      self.live_posj_display_var = create_display_entry(self.root, "수신 대기중...", current_row, 1, entry_width=60, readonly=True)
      tk.Button(self.root, text="복사", command=lambda: copy_to_clipboard(self.root, self.live_posj_display_var)).grid(row=current_row, column=2, padx=5, pady=2)
    
      current_row += 1
      tk.Label(self.root, text="캡처된 3점 좌표:", font=('Arial', 10, 'bold')).grid(row=current_row, column=0, columnspan=3, sticky="w", padx=5, pady=(10,0))
      for i in range(3):
          current_row += 1
          tk.Label(self.root, text=self.point_labels_text[i]).grid(row=current_row, column=0, sticky="w", padx=10, pady=2)
          display_var = create_display_entry(self.root, "캡처 대기중...", current_row, 1, entry_width=60, readonly=True)
          self.captured_point_display_vars.append(display_var)
          tk.Button(self.root, text=f"{self.point_labels_text[i].split(' ')[0]} 캡처",
                    command=lambda idx=i: self.capture_point(idx)).grid(row=current_row, column=2, padx=5, pady=2)
    
      current_row += 1
      # 변경점: 버튼 텍스트 변경 및 강조색 변경
      self.save_and_calc_button = tk.Button(self.root, text="CSV 저장 및 원 중심/반지름 계산", command=self.perform_save_and_calculate, bg="lightgreen", font=('Arial', 10, 'bold'))
      self.save_and_calc_button.grid(row=current_row, column=0, columnspan=3, padx=5, pady=10, sticky="ew")




      current_row += 1
      tk.Label(self.root, text="계산된 원 중심(XYZ):", font=('Arial', 10, 'bold')).grid(row=current_row, column=0, columnspan=3, sticky="w", padx=5, pady=(5,0))
      current_row += 1
      self.calculated_center_display_var = create_display_entry(self.root, "계산 대기중...", current_row, 1, entry_width=60, readonly=True)
      tk.Button(self.root, text="복사", command=lambda: copy_to_clipboard(self.root, self.calculated_center_display_var)).grid(row=current_row, column=2, padx=5, pady=2)




      # >>> 추가된 부분: 반지름 표시 GUI 요소 <<<
      current_row += 1
      tk.Label(self.root, text="계산된 원 반지름:", font=('Arial', 10, 'bold')).grid(row=current_row, column=0, columnspan=3, sticky="w", padx=5, pady=(5,0))
      current_row += 1
      self.calculated_radius_display_var = create_display_entry(self.root, "계산 대기중...", current_row, 1, entry_width=60, readonly=True)
      tk.Button(self.root, text="복사", command=lambda: copy_to_clipboard(self.root, self.calculated_radius_display_var)).grid(row=current_row, column=2, padx=5, pady=2)




      # 기존 코드 뒤, status_message_var 아래에 버튼 추가 부분
      current_row += 1


       # P3 -> P1 이동 버튼 추가 (execute_robot() 호출)
      move_p3_to_p1_button = tk.Button(
           self.root,
           text="home",
           command=execute_robot, 
           bg="lightblue",
           font=('Arial', 10, 'bold')
       )
      move_p3_to_p1_button.grid(row=current_row, column=0, columnspan=3, sticky="ew", padx=5, pady=10)


      # >>> END 추가된 부분 <<<




      current_row += 1
      self.status_message_var = StringVar()
      self.status_message_var.set("준비 완료. ROS2 토픽 구독자 시작 중...")
      status_label = tk.Label(self.root, textvariable=self.status_message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
      status_label.grid(row=current_row, column=0, columnspan=3, sticky="ew", padx=5, pady=5)




      self.ros_subscriber_node = PosTopicSubscriber(self.live_posx_display_var, self.live_posj_display_var)
      self.ros_subscriber_thread = threading.Thread(target=lambda: rclpy.spin(self.ros_subscriber_node), daemon=True)
      self.ros_subscriber_thread.start()
      self.update_status("준비 완료. 로봇 좌표 수신 대기중...")
    
      self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
      self.set_initial_robot_mode() # 로봇 모드 설정 시도




  # 추가된 부분: 로봇 모드 설정 응답 처리를 위한 콜백
  def robot_mode_set_callback(self, future):
      try:
          response = future.result()
          if response and response.success:
              self.update_status(f"로봇 모드 설정 성공 (응답).")
          else:
              self.update_status(f"로봇 모드 설정 실패 (응답): {response.message if response else '응답 없음'}.")
      except Exception as e:
          self.update_status(f"로봇 모드 설정 응답 처리 중 오류: {e}")




  def set_initial_robot_mode(self): # 로봇 모드 설정 함수
      if self.service_client_node:
          self.update_status("초기 로봇 모드 설정 시도 (예: 수동 모드 0)...")
          future = self.service_client_node.send_robot_mode_request(0)
          if future:
              # 변경점: 비동기 결과 처리를 위해 콜백 추가
              future.add_done_callback(self.robot_mode_set_callback)
              self.update_status("로봇 모드 설정 요청 전송됨. 응답 대기 중...")
          else:
              self.update_status("로봇 모드 설정 서비스 호출에 즉시 실패했습니다.")
      else:
          self.update_status("서비스 클라이언트 노드가 없어 로봇 모드를 설정할 수 없습니다.")








  def update_status(self, message): # 변경 없음
      self.status_message_var.set(message)




  def capture_point(self, point_index_to_capture): # 변경 없음
      current_posx_string = self.live_posx_display_var.get()
      self.update_status(f"{self.point_labels_text[point_index_to_capture].split(' ')[0]} 캡처 시도...")
      try:
          if not current_posx_string or not current_posx_string.startswith("posx(") or not current_posx_string.endswith(")"):
              raise ValueError("실시간 좌표 문자열 형식이 올바르지 않습니다.")
          coord_list_as_string = current_posx_string[len("posx("):-1].strip('[]')
          if not coord_list_as_string or coord_list_as_string == "수신 대기중...":
               raise ValueError("수신된 유효한 좌표값이 없습니다.")
          coords_as_float = [float(c.strip()) for c in coord_list_as_string.split(',')]
          if len(coords_as_float) != 6:
              raise ValueError(f"6개의 좌표값(posx)이 필요하지만 {len(coords_as_float)}개를 얻었습니다.")
          self.captured_points_posx[point_index_to_capture] = coords_as_float
          display_text = f"posx({[round(c, 3) for c in coords_as_float]})"
          self.captured_point_display_vars[point_index_to_capture].set(display_text)
          self.update_status(f"{self.point_labels_text[point_index_to_capture].split(' ')[0]} 캡처 완료.")
      except ValueError as ve:
          self.update_status(f"좌표 파싱 오류: {ve} (원본: '{current_posx_string}')")
          messagebox.showerror("캡처 오류", f"좌표 파싱 중 오류 발생:\n{ve}\n원본 문자열: '{current_posx_string}'")
      except Exception as e:
          self.update_status(f"캡처 중 알 수 없는 오류: {e} (원본: '{current_posx_string}')")
          messagebox.showerror("캡처 오류", f"알 수 없는 오류 발생:\n{e}")




  def perform_save_and_calculate(self): # 변경점: 반지름 처리 추가
      if any(p_data is None for p_data in self.captured_points_posx):
          messagebox.showerror("오류", "모든 3개의 점을 먼저 캡처해야 합니다.")
          self.update_status("오류: 모든 3개의 점을 먼저 캡처해야 합니다.")
          return




      default_filename = "captured_robot_points.csv"
      filepath_to_save = filedialog.asksaveasfilename(
          defaultextension=".csv", initialfile=default_filename, title="캡처된 좌표를 CSV 파일로 저장",
          filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
      )
      if not filepath_to_save:
          self.update_status("CSV 파일 저장이 취소되었습니다.")
          return




      try:
          with open(filepath_to_save, 'w', newline='', encoding='utf-8') as csv_file:
              csv_data_writer = csv.writer(csv_file)
              csv_data_writer.writerow(['Point_ID', 'X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'])
              point_ids = [name.split(' ')[0] for name in self.point_labels_text]
              for i, single_point_data in enumerate(self.captured_points_posx):
                  csv_data_writer.writerow([point_ids[i]] + single_point_data)
          self.update_status(f"좌표를 '{os.path.basename(filepath_to_save)}' 에 성공적으로 저장했습니다.")
      except IOError as e_io:
          messagebox.showerror("CSV 저장 오류", f"파일 저장 중 오류 발생: {e_io}")
          self.update_status(f"CSV 파일 저장 오류: {e_io}")
          return




      points_xyz_for_calc = [p_data[:3] for p_data in self.captured_points_posx]
      self.update_status("원의 중심 및 반지름 계산 중...")
    
      # 변경점: 함수 호출 변경 및 반환 값 두 개 받기
      calculated_center_xyz, calculated_radius = calculate_circle_center_and_radius_for_gui(
          points_xyz_for_calc[0], points_xyz_for_calc[1], points_xyz_for_calc[2],
          status_update_func=self.update_status
      )




      if calculated_center_xyz and calculated_radius is not None: # 변경점: 반지름도 None이 아닌지 확인
          center_xyz_rounded = [round(c, 3) for c in calculated_center_xyz]
          radius_rounded = round(calculated_radius, 3) # << 추가: 반지름 반올림




          self.calculated_center_display_var.set(str(center_xyz_rounded))
          self.calculated_radius_display_var.set(str(radius_rounded)) # << 추가: 반지름 GUI에 표시
          self.update_status(f"원 중심: {center_xyz_rounded}, 반지름: {radius_rounded}") # << 변경: 상태 메시지에 반지름 추가
      else:
          self.calculated_center_display_var.set("계산 실패")
          self.calculated_radius_display_var.set("계산 실패") # << 추가: 반지름도 실패로 표시
          # 오류 메시지는 계산 함수가 이미 status_update_func를 통해 표시했을 것임




  def on_closing(self): # 변경 없음
      if messagebox.askokcancel("종료 확인", "프로그램을 종료하시겠습니까?"):
          self.update_status("애플리케이션 종료 중...")
          if hasattr(self, 'ros_subscriber_node') and self.ros_subscriber_node and rclpy.ok():
              self.ros_subscriber_node.destroy_node()
          if self.service_client_node and rclpy.ok():
              self.service_client_node.destroy_node()
          self.root.destroy()




def execute_robot():
   print("home")
   is_success = threading.Thread(target = movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)).start()
   mwait()
   print("is successed? : ", is_success)
   return




def main_gui_entry_point(): # 변경 없음 (내부 로직은 이전과 동일)
  app_service_client_node = ServiceClientNode("main_app_service_client")
  root = tk.Tk()
  app = RobotPointCaptureApp(root, app_service_client_node)
  try:
      root.mainloop()
  except KeyboardInterrupt:
      print("Tkinter GUI가 KeyboardInterrupt로 종료됨.")
  finally:
      print("ROS2 시스템 종료 중...")
      if app_service_client_node and rclpy.ok():
           app_service_client_node.destroy_node()
      if hasattr(app, 'ros_subscriber_node') and app.ros_subscriber_node and rclpy.ok(): # app 객체 존재 확인
           app.ros_subscriber_node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()
      print("ROS2 시스템 종료 완료.")




if __name__ == '__main__':
  main_gui_entry_point()













