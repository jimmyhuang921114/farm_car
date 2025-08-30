# import sys
# import rclpy
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.node import Node
# import serial
# import threading
# from std_msgs.msg import Float32
# import time
# import traceback

# ID_TO_SERIAL_PORT = {
#     0: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02619997-if00-port0',
#     1: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0111621A-if00-port0',
#     2: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0',
#     3: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199F2-if00-port0',
#     4: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF44C-if00-port0',
#     5: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF47D-if00-port0',
#     6: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF471-if00-port0'
# }


# class UWBReceiver(Node):
#     def __init__(self, id, serial_port):
#         super().__init__(f'uwb_receiver_node_{id}')
#         self.id = id
#         self.serial_port = serial_port
#         self.publisher_ = self.create_publisher(Float32, f'uwb_raw_data_{id}', 10)
#         self.ser = None
#         self.reconnect_serial()

#         threading.Thread(target=self.read_from_serial, daemon=True).start()

#     def reconnect_serial(self):
#         while self.ser is None:
#             try:
#                 self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
#                 self.get_logger().info(f"Connected to serial port: {self.serial_port}")
#             except serial.SerialException as e:
#                 self.get_logger().error(f"Serial connection failed: {e}. Retrying in 5 seconds...")
#                 time.sleep(5)

#     def read_from_serial(self):
#         buffer = ""
#         while True:
#             try:
#                 if self.ser and self.ser.in_waiting > 0:
#                     chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
#                     buffer += chunk
#                     lines = buffer.split('\r\n')

#                     for line in lines[:-1]:
#                         try:
#                             distance_str = line.split(":")[1].strip().replace(' cm', '')
#                             distance = float(distance_str)
#                             if 0 <= distance <= 2000:  # 合理範圍
#                                 self.publish_distance(distance)
#                         except (ValueError, IndexError):
#                             self.get_logger().warning(f"Received invalid data: {line}")
#                             continue

#                     buffer = lines[-1]
#             except (serial.SerialException, OSError) as e:
#                 self.get_logger().error(f"Serial error: {e}. Attempting to reconnect...")
#                 self.ser = None
#                 self.reconnect_serial()
#             except Exception as e:
#                 self.get_logger().error(f"Unexpected error in read_from_serial: {e}")
#                 self.get_logger().error(traceback.format_exc())
#                 time.sleep(5)

#     def publish_distance(self, distance):
#         msg = Float32()
#         msg.data = round(distance, 2)
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published raw UWB data from ID {self.id}: Distance = {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)

#     if len(sys.argv) < 2:
#         print("請指定要使用的 UWB ID，例如：ros2 run uwb_receiver 0")
#         sys.exit(1)

#     ids_to_use = [int(id) for id in sys.argv[1:] if id.isdigit()]
#     nodes = [UWBReceiver(id, ID_TO_SERIAL_PORT[id]) for id in ids_to_use]

#     executor = MultiThreadedExecutor()
#     for node in nodes:
#         executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user.")
#     finally:
#         for node in nodes:
#             node.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import sys
import rclpy
from rclpy.node import Node
import serial
import time
import traceback
from std_msgs.msg import Float32MultiArray
from math import isnan, nan

# 只監控這幾個 UWB ID
IDS = [0, 2, 3, 5]
TIMEOUT_SEC = 0.6  # 超過這段秒數沒更新就視為過期

ID_TO_SERIAL_PORT = {
    0: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02619997-if00-port0',
    1: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0111621A-if00-port0',
    2: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0',
    3: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199F2-if00-port0',
    4: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF44C-if00-port0',
    5: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF47D-if00-port0',
    6: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF471-if00-port0'
}

class UWB(Node):
    def __init__(self):
        super().__init__('UWB')
        # serial 連線、buffer
        self.sers    = {i: None for i in IDS}
        self.buffers = {i: ''   for i in IDS}
        # 最新距離與最後更新時間
        self.latest      = {i: nan for i in IDS}
        self.last_update = {i: 0.0 for i in IDS}

        self.pub = self.create_publisher(Float32MultiArray, '/uwb/raw_data', 10)
        # 以 0.05 秒頻率讀資料並發佈
        self.create_timer(0.12, self.timer_callback)

    def timer_callback(self):
        now = time.time()

        for i in IDS:
            # 確保 serial 已連
            if self.sers[i] is None:
                try:
                    self.sers[i] = serial.Serial(ID_TO_SERIAL_PORT[i], 115200, timeout=0)
                    self.get_logger().info(f"ID{i} 連線成功")
                except serial.SerialException:
                    continue

            ser = self.sers[i]
            try:
                # 讀取可用資料
                if ser.in_waiting > 0:
                    chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    self.buffers[i] += chunk
                    lines = self.buffers[i].split('\r\n')
                    # 處理完整行
                    for line in lines[:-1]:
                        try:
                            dist = float(line.split(':')[1].replace(' cm','').strip())
                            if 0 <= dist <= 2000:
                                prev = self.latest[i]
                                MAX_JUMP = 200  # 單次最大允許跳變距離（單位：cm）
                                
                                # 若為首次數據 or 跳變合理，才接受
                                if isnan(prev) or abs(dist - prev) <= MAX_JUMP:
                                    self.latest[i] = round(dist, 2)
                                    self.last_update[i] = now
                                else:
                                    self.get_logger().warn(f"ID{i} 距離跳變過大，捨棄值: {dist:.2f} (前值: {prev:.2f})")

                        except Exception:
                            pass
                    # 保留殘留不完整行
                    self.buffers[i] = lines[-1]
            except Exception as e:
                self.get_logger().error(f"ID{i} 讀取錯誤: {e}")
                try: ser.close()
                except: pass
                self.sers[i] = None

        # 組出要發佈的 list：若超時則為 None
        pub_list = []
        for i in IDS:
            if now - self.last_update[i] > TIMEOUT_SEC:
                pub_list.append(None)
            else:
                pub_list.append(self.latest[i])

        # 印出與發佈（None 會轉成 NaN）
        self.get_logger().info(f"同步距離列表: {pub_list}")
        msg = Float32MultiArray()
        msg.data = [float(x) if x is not None else nan for x in pub_list]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UWB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
