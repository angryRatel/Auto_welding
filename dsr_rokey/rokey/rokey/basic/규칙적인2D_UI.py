import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


import tkinter as tk
from tkinter import StringVar
import threading


from dsr_msgs2.srv import SetRobotMode




def create_entries(root, default_value, row, col):
   entry_var = StringVar()
   entry_var.set(str(round(default_value, 3)))
   entry = tk.Entry(root, textvariable=entry_var, width=50)
   entry.grid(row=row, column=col, padx=10, pady=5)
   return entry_var




class ServiceClinetNode(Node):
   def __init__(self):
       super().__init__("service_client_node")
       self.cli = self.create_client(SetRobotMode, "/dsr01/system/set_robot_mode")
       while not self.cli.wait_for_service(timeout_sec=1.0):
           print("Waiting for service...")


   def send_request(self, mode=0):
       request = SetRobotMode.Request()
       request.robot_mode = mode
       future = self.cli.call_async(request)
       rclpy.spin_until_future_complete(self, future)
       return future.result()




class PosTopicSubscriber(Node):
   def __init__(self):
       super().__init__("PosTopicSubscriber")


       self.current_posx_msg = None
       self.posx_2 = []
       self.posx_3 = []


       # publisher 생성
       self.publisher_ = self.create_publisher(Float64MultiArray, "/saved_posx_pair", 10)


       self.create_subscription(
           Float64MultiArray, "/dsr01/msg/current_posx", self.current_posx_callback, 10
       )


   def current_posx_callback(self, msg):
       self.current_posx_msg = msg


   def save_posx_to_var1(self, text_var):
       if self.current_posx_msg is not None:
           self.posx_2 = [round(d, 3) for d in self.current_posx_msg.data]
           text_var.set(f"posx_2({self.posx_2})")


   def save_posx_to_var2(self, text_var):
       if self.current_posx_msg is not None:
           self.posx_3 = [round(d, 3) for d in self.current_posx_msg.data]
           text_var.set(f"posx_3({self.posx_3})")


   def publish_saved_posx_pair(self):
       if self.posx_2 and self.posx_3:
           combined = self.posx_2 + self.posx_3
           msg = Float64MultiArray()
           msg.data = combined
           self.publisher_.publish(msg)
           print("Published posx_2 + posx_3:", combined)
       else:
           print("Both posx_2 and posx_3 must be saved before publishing.")




def ros_thread(save_callbacks_holder):
   node = PosTopicSubscriber()
   save_callbacks_holder.append(node)  # 노드 인스턴스 전달


   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()




# ... (기존 코드 생략)


def main():
   root = tk.Tk()


   # 첫 번째 posx 저장용
   tk.Label(root, text="saved posx 1:").grid(row=0, column=0)
   text_var1 = create_entries(root, 0.0, 0, 1)
   # 두 번째 posx 저장용
   tk.Label(root, text="saved posx 2:").grid(row=1, column=0)
   text_var2 = create_entries(root, 0.0, 1, 1)


   # 노드 콜백 참조용
   save_callbacks_holder = []


   # 버튼 콜백
   def save_posx1():
       if save_callbacks_holder:
           node = save_callbacks_holder[0]
           node.save_posx_to_var1(text_var1)


   def save_posx2():
       if save_callbacks_holder:
           node = save_callbacks_holder[0]
           node.save_posx_to_var2(text_var2)


   def start_action():
       if save_callbacks_holder:
           node = save_callbacks_holder[0]
           node.publish_saved_posx_pair()


   # 저장 버튼들
   tk.Button(root, text="save posx (1)", command=save_posx1).grid(row=0, column=2, padx=2, pady=5)
   tk.Button(root, text="save posx (2)", command=save_posx2).grid(row=1, column=2, padx=2, pady=5)


   # 아래에 '시작' 버튼 추가
   tk.Button(root, text="시작", command=start_action).grid(row=2, column=1, pady=10)


   # 서비스 실행
   print("Service Start")
   rclpy.init()
   client_node = ServiceClinetNode()
   response = client_node.send_request(0)
   client_node.get_logger().info(f"results: {response}")


   # ROS2 스레드 실행
   ros = threading.Thread(target=ros_thread, args=(save_callbacks_holder,))
   ros.start()


   root.mainloop()






if __name__ == "__main__":
   main()





