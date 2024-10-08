from serial import Serial
import json
from pathlib import Path

from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RC_Receiver(Node):
    
    def __init__(self):
        super().__init__('rc_receiver')
        # print("Directory Path:", Path().absolute())
        self.publisher_ = self.create_publisher(String, 'controller', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        
        file = open('rc_receiver_config.json', 'r')    
        config = json.load(file)
        port = config["port"]
        self.get_logger().info('Using serial port: "%s"' %port)
        self.serial = Serial(port, 115200)
        
        self.get_logger().info('Init finish')
        self.input_buf = b''
        self.serial_handle()
            
    def serial_handle(self):
        while True:
            # Serial has data
            if(self.serial.in_waiting > 0):
                self.get_logger().debug(f"receive count: {self.serial.in_waiting}")
                self.input_buf += self.serial.read_all()
            # Input buffer >= 2
            if len(self.input_buf) >= 2:
                self.get_logger().debug(f"buffer size: {len(self.input_buf)}")
                # Find header
                start = self.input_buf.find(b'\x20\x40')
                self.get_logger().debug(f"start: {start}")
                # Header not found
                if start == -1:
                    continue
                # Header not at index 0 (unsync)
                if start != 0:
                    self.get_logger().warning(f"Unsync detected")
                # Sync with header
                self.input_buf = self.input_buf[start:]
                # Input buffer >= 32
                if len(self.input_buf) >= 32:
                    # Move input buffer packet to local variable
                    raw_data = self.input_buf[:32]
                    self.input_buf = self.input_buf[32:]
                    data = list(raw_data)
                    # Calculate checksum
                    sum = 0
                    for idx in range(0, 30):
                        sum += data[idx]
                    sum ^= 0xFFFF
                    checksum = data[30] + (data[31] << 8)
                    # Checksum failed
                    if sum != checksum:
                        self.get_logger().warning(f"Checksum failed {sum}/{checksum}")
                        continue
                    # Parse to channel data
                    output_data = [0] * 18
                    output_data[0] = data[2] + ((data[3] & 0x0F) << 8)
                    output_data[1] = data[4] + ((data[5] & 0x0F) << 8)
                    output_data[2] = data[6] + ((data[7] & 0x0F) << 8)
                    output_data[3] = data[8] + ((data[9] & 0x0F) << 8)
                    output_data[4] = data[10] + ((data[11] & 0x0F) << 8)
                    output_data[5] = data[12] + ((data[13] & 0x0F) << 8)
                    output_data[6] = data[14] + ((data[15] & 0x0F) << 8)
                    output_data[7] = data[16] + ((data[17] & 0x0F) << 8)
                    output_data[8] = data[18] + ((data[19] & 0x0F) << 8)
                    output_data[9] = data[20] + ((data[21] & 0x0F) << 8)
                    output_data[10] = data[22] + ((data[23] & 0x0F) << 8)
                    output_data[11] = data[24] + ((data[25] & 0x0F) << 8)
                    output_data[12] = data[26] + ((data[27] & 0x0F) << 8)
                    output_data[13] = data[28] + ((data[29] & 0x0F) << 8)
                    output_data[14] = ((data[3] & 0xF0) >> 4) + (data[5] & 0xF0) + ((data[7] & 0xF0) << 4)
                    output_data[15] = ((data[9] & 0xF0) >> 4) + (data[11] & 0xF0) + ((data[13] & 0xF0) << 4)
                    output_data[16] = ((data[15] & 0xF0) >> 4) + (data[17] & 0xF0) + ((data[19] & 0xF0) << 4)
                    output_data[17] = ((data[21] & 0xF0) >> 4) + (data[23] & 0xF0) + ((data[25] & 0xF0) << 4)
                    # print(data)
                    print(output_data)
                

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)
    rc_receiver = RC_Receiver()
    rclpy.spin(rc_receiver)
    rc_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
