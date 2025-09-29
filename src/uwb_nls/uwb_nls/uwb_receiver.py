import sys
import glob
import os
import re
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import math
import time
import traceback

# === 取最小「連號」的 CH9344 連接埠 ===
def discover_ch9344_ports_consecutive(required_count=4):
    """
    掃描 /dev/ttyCH9344USB*，找出最小號碼 min_n，
    並強制回傳 [min_n, min_n+1, ..., min_n+required_count-1] 這組連號的裝置路徑。
    若任何一個缺少，則回傳空清單（由呼叫者決定報錯）。
    """
    paths = glob.glob("/dev/ttyCH9344USB*")
    nums = []
    for p in paths:
        m = re.search(r"/dev/ttyCH9344USB(\d+)$", p)
        if m:
            nums.append(int(m.group(1)))
    if not nums:
        return []

    min_n = min(nums)
    expected = [f"/dev/ttyCH9344USB{min_n + i}" for i in range(required_count)]

    for dev in expected:
        if not os.path.exists(dev):
            # 有缺就視為不符合「連號四個」的要求
            return []
    return expected

def build_id_to_port(ids_in_order, ports_in_order):
    """
    依「使用者提供的邏輯 ID 順序」對應到「已選好的連號埠順序」。
    例如 ids_in_order=[0,2,3,5], ports_in_order=[p2,p3,p4,p5] =>
        {0:p2, 2:p3, 3:p4, 5:p5}
    """
    mapping = {}
    if len(ids_in_order) > len(ports_in_order):
        ids_in_order = ids_in_order[:len(ports_in_order)]
    for idx, logic_id in enumerate(ids_in_order):
        mapping[logic_id] = ports_in_order[idx]
    return mapping

class UWBReceiverMulti(Node):
    """
    多埠讀取 → 單一 Float32MultiArray 發佈
    - Topic: /uwb_raw_data
    - 順序：依 ids_in_order
    - 任何一個感測器更新就發一次（也可改成固定頻率）
    """
    def __init__(self, ids_in_order, id_to_port):
        super().__init__('uwb_receiver_multi_node')
        self.ids_in_order = list(ids_in_order)
        self.id_to_port = dict(id_to_port)

        # Publisher: Float32MultiArray
        self.pub = self.create_publisher(Float32MultiArray, '/uwb/raw_data', 10)

        # 最新數值（以 NaN 起始，表示尚未讀到）
        self.latest = {logic_id: math.nan for logic_id in self.ids_in_order}
        self.latest_lock = threading.Lock()

        # 每個埠各起一條執行緒讀資料
        self.threads = []
        for logic_id in self.ids_in_order:
            t = threading.Thread(
                target=self._reader_loop,
                args=(logic_id, self.id_to_port[logic_id]),
                daemon=True
            )
            t.start()
            self.threads.append(t)

        self.get_logger().info("UWBReceiverMulti started.")
        self.get_logger().info("IDs order: " + ", ".join(map(str, self.ids_in_order)))

    # === 串口讀取迴圈（每個感測器一條執行緒） ===
    def _reader_loop(self, logic_id, port_path):
        ser = None
        buffer = ""

        def try_connect():
            nonlocal ser
            while rclpy.ok() and ser is None:
                try:
                    ser = serial.Serial(port_path, 9600, timeout=1)
                    self.get_logger().info(f"[ID {logic_id}] Connected to serial port: {port_path}")
                except serial.SerialException as e:
                    self.get_logger().error(f"[ID {logic_id}] Serial connection failed: {e}. Retrying in 5 seconds...")
                    time.sleep(5)

        try_connect()

        while rclpy.ok():
            try:
                if ser and ser.in_waiting > 0:
                    chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    lines = buffer.split('\r\n')

                    for line in lines[:-1]:
                        # 預期格式："...: <number> cm"
                        try:
                            distance_str = line.split(":")[1].strip().replace(' cm', '')
                            distance = float(distance_str)
                            if 0.0 <= distance <= 2000.0:
                                self._update_and_publish(logic_id, round(distance, 2))
                            else:
                                self.get_logger().warning(f"[ID {logic_id}] Out-of-range data: {distance}")
                        except (ValueError, IndexError):
                            self.get_logger().warning(f"[ID {logic_id}] Received invalid data: {line}")
                            continue

                    buffer = lines[-1]
                else:
                    time.sleep(0.005)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"[ID {logic_id}] Serial error: {e}. Attempting to reconnect...")
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                ser = None
                try_connect()
            except Exception as e:
                self.get_logger().error(f"[ID {logic_id}] Unexpected error: {e}")
                self.get_logger().error(traceback.format_exc())
                time.sleep(5)

    # === 更新某感測器值並馬上組陣列發佈 ===
    def _update_and_publish(self, logic_id, value):
        with self.latest_lock:
            self.latest[logic_id] = value
            # 組成 Float32MultiArray，順序照 ids_in_order
            data = [float(self.latest[i]) for i in self.ids_in_order]

        msg = Float32MultiArray()
        # 設定 layout（可選，但有助除錯）
        dim = MultiArrayDimension()
        dim.label = "sensors"
        dim.size = len(self.ids_in_order)
        dim.stride = len(self.ids_in_order)
        layout = MultiArrayLayout(dim=[dim], data_offset=0)
        msg.layout = layout
        msg.data = data

        self.pub.publish(msg)
        self.get_logger().info(f"Published UWB array: {data}")

def parse_ids_from_argv(argv):
    """
    從命令列參數取邏輯 ID（接受任意整數字串），若沒提供則預設 [0,1,2,3]
    保持「輸入順序」以便 mapping。
    """
    ids = []
    for x in argv:
        if re.fullmatch(r"-?\d+", x):
            ids.append(int(x))
    if not ids:
        ids = [0, 1, 2, 3]
    return ids

def main(args=None):
    rclpy.init(args=args)

    # 你可以改這裡的邏輯 ID 順序
    ids_in_order = [0, 2, 3, 5]
    required_count = len(ids_in_order)

    # 取最小連號的實體埠（長度需與 ids 相同）
    ports_in_order = discover_ch9344_ports_consecutive(required_count=required_count)
    if len(ports_in_order) != required_count:
        print(f"找不到連號的 CH9344 串口：需要連號 {required_count} 個。")
        print("請確認 /dev/ttyCH9344USB* 是否連續存在（例如 2,3,4,5）以及權限（dialout 群組）。")
        rclpy.shutdown()
        sys.exit(1)

    # 依輸入的邏輯 ID 順序做一一對應
    id_to_port = build_id_to_port(ids_in_order, ports_in_order)

    print("使用的串口對應：")
    for logic_id in ids_in_order:
        print(f"  ID {logic_id} -> {id_to_port[logic_id]}")

    node = UWBReceiverMulti(ids_in_order=ids_in_order, id_to_port=id_to_port)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import sys
# import rclpy
# from rclpy.node import Node
# import serial
# import time
# import traceback
# from std_msgs.msg import Float32MultiArray
# from math import isnan, nan

# # 只監控這幾個 UWB ID
# IDS = [0, 2, 3, 5]
# TIMEOUT_SEC = 0.6  # 超過這段秒數沒更新就視為過期

# ID_TO_SERIAL_PORT = {
#     0: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02619997-if00-port0',
#     1: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0111621A-if00-port0',
#     2: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0',
#     3: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199F2-if00-port0',
#     4: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF44C-if00-port0',
#     5: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF47D-if00-port0',
#     6: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF471-if00-port0'
# }

# class UWB(Node):
#     def __init__(self):
#         super().__init__('UWB')
#         # serial 連線、buffer
#         self.sers    = {i: None for i in IDS}
#         self.buffers = {i: ''   for i in IDS}
#         # 最新距離與最後更新時間
#         self.latest      = {i: nan for i in IDS}
#         self.last_update = {i: 0.0 for i in IDS}

#         self.pub = self.create_publisher(Float32MultiArray, '/uwb/raw_data', 10)
#         # 以 0.05 秒頻率讀資料並發佈
#         self.create_timer(0.12, self.timer_callback)

#     def timer_callback(self):
#         now = time.time()

#         for i in IDS:
#             # 確保 serial 已連
#             if self.sers[i] is None:
#                 try:
#                     self.sers[i] = serial.Serial(ID_TO_SERIAL_PORT[i], 115200, timeout=0)
#                     self.get_logger().info(f"ID{i} 連線成功")
#                 except serial.SerialException:
#                     continue

#             ser = self.sers[i]
#             try:
#                 # 讀取可用資料
#                 if ser.in_waiting > 0:
#                     chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
#                     self.buffers[i] += chunk
#                     lines = self.buffers[i].split('\r\n')
#                     # 處理完整行
#                     for line in lines[:-1]:
#                         try:
#                             dist = float(line.split(':')[1].replace(' cm','').strip())
#                             if 0 <= dist <= 2000:
#                                 prev = self.latest[i]
#                                 MAX_JUMP = 200  # 單次最大允許跳變距離（單位：cm）
                                
#                                 # 若為首次數據 or 跳變合理，才接受
#                                 if isnan(prev) or abs(dist - prev) <= MAX_JUMP:
#                                     self.latest[i] = round(dist, 2)
#                                     self.last_update[i] = now
#                                 else:
#                                     self.get_logger().warn(f"ID{i} 距離跳變過大，捨棄值: {dist:.2f} (前值: {prev:.2f})")

#                         except Exception:
#                             pass
#                     # 保留殘留不完整行
#                     self.buffers[i] = lines[-1]
#             except Exception as e:
#                 self.get_logger().error(f"ID{i} 讀取錯誤: {e}")
#                 try: ser.close()
#                 except: pass
#                 self.sers[i] = None

#         # 組出要發佈的 list：若超時則為 None
#         pub_list = []
#         for i in IDS:
#             if now - self.last_update[i] > TIMEOUT_SEC:
#                 pub_list.append(None)
#             else:
#                 pub_list.append(self.latest[i])

#         # 印出與發佈（None 會轉成 NaN）
#         self.get_logger().info(f"同步距離列表: {pub_list}")
#         msg = Float32MultiArray()
#         msg.data = [float(x) if x is not None else nan for x in pub_list]
#         self.pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = UWB()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

