import rclpy
import DR_init
from std_msgs.msg import Float64MultiArray


# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# 전역 변수
posx_2 = None
posx_3 = None
received = False  # 데이터 수신 여부




def callback(msg):
   global posx_2, posx_3, received
   if received:
       return  # 한 번만 받음


   data = msg.data
   if len(data) >= 12:
       from DR_common2 import posx
       posx_2 = posx(list(data[:6]))
       posx_3 = posx(list(data[6:12]))
       received = True
       print("[INFO] posx_2 and posx_3 received.")
   else:
       print("[WARN] Received data length too short.")




def main(args=None):
   global posx_2, posx_3, received


   rclpy.init(args=args)
   node = rclpy.create_node("posx_mover", namespace=ROBOT_ID)
   DR_init.__dsr__node = node


   from DSR_ROBOT2 import set_tool, set_tcp, movej, movel
   from DR_common2 import posj


   JReady = [0, 0, 90, 0, 90, 0]


   set_tool("Tool Weight_2FG")
   set_tcp("2FG_TCP")


   # ROS2 토픽 구독 시작
   node.create_subscription(
       Float64MultiArray,
       "/saved_posx_pair",
       callback,
       10
   )


   print("[INFO] Waiting for posx data...")
   while rclpy.ok() and not received:
       rclpy.spin_once(node, timeout_sec=0.1)


   if received and posx_2 and posx_3:
       print("[INFO] Starting motion sequence...")
       while rclpy.ok() and received:
           print("→ movej JReady")
           movej(JReady, vel=VELOCITY, acc=ACC)


           print("→ movel posx_2")
           movel(posx_2, vel=VELOCITY, acc=ACC)


           print("→ movel posx_3")
           movel(posx_3, vel=VELOCITY, acc=ACC)


           received = False
   else:
       print("[ERROR] No posx data received. Exiting.")


   rclpy.shutdown()




if __name__ == "__main__":
   main()





