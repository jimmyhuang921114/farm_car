from serial import Serial
import json
from pathlib import Path
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int16MultiArray
from rc_receiver_if.msg import ReceiverData
from geometry_msgs.msg import Twist


DUMMY_RECEIVER = False


class RC_Receiver(Node):
    def __init__(self):
        super().__init__('rc_receiver')
        # print("Directory Path:", Path().absolute())
        self.channel_data = [1000] * 18
        self.twist = Twist()

        self.pub_raw = self.create_publisher(ReceiverData, '/rc_receiver/raw', 1)
        self.pub_twist = self.create_publisher(Twist, '/motor/main', 1)
        self.signal_alart_pub = self.create_subscription(Int16MultiArray,'visual_signal',self.visual_signal,1)
        self.sub_loopback = self.create_subscription(ReceiverData, '/rc_receiver', self.loopbackCallback, 1)
        self.timeout_timer = self.create_timer(3, self.timeoutTimerCallback)
        self.signal_alart = 1
        if not DUMMY_RECEIVER:
            file = open('rc_receiver_config.json', 'r')    
            config = json.load(file)
            port = config["port"]
            self.get_logger().info('Using serial port: "%s"' %port)
            try:
                self.serial = Serial(port, 115200)
            except Exception as e:
                self.get_logger().error(f'Failed to open port, error:\n\t{e}')
                exit()
        
        self.get_logger().info('Init finish')
        self.input_buf = b''
        self.serial_handle_timer = self.create_timer(0.01, self.serialHandleTimerCallback)
        self.publish_timer = self.create_timer(0.01, self.publishTimerCallback)


    def parseData(self, data: list) -> bool:
        # Calculate checksum
        sum = 0
        for idx in range(0, 30):
            sum += data[idx]
        sum ^= 0xFFFF
        checksum = data[30] + (data[31] << 8)
        # Checksum failed
        if sum != checksum:
            self.get_logger().warning(f"Checksum failed {sum}/{checksum}")
            return False
        # Parse to channel data
        output_data = [1000] * 18
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
        # print(output_data)
        self.get_logger().debug(f"output data: {output_data}")
        self.channel_data = output_data
        linear_vel = (output_data[1] - 1500) * 0.001
        ang_vel = -(output_data[0] - 1500) * 0.001
        self.twist.angular.z = ang_vel
        self.twist.linear.z = linear_vel
        return True
    
    def visual_signal(self,msg):
        self.signal_alart = msg()
        if self.signal_alart==1:
            self.get_logger().init(self.signal_alart)
        elif self.signal_alart==0:
            self.get_logger().error(self.signal_alart)
    def serialHandleTimerCallback(self):
        # Serial has data, read into buffer
        if(not DUMMY_RECEIVER and self.serial.in_waiting > 0):
            self.get_logger().debug(f"receive count: {self.serial.in_waiting}")
            self.input_buf += self.serial.read_all()
        # Input buffer >= 2
        if len(self.input_buf) >= 2:
            self.get_logger().debug(f"buffer size: {len(self.input_buf)}")
            # Find header
            start = self.input_buf.find(b'\x20\x40')
            self.get_logger().debug(f"start: {start}")
            # Header not found, skip
            if start == -1:
                return
            # Header not at index 0 (unsync), log warning
            if start != 0:
                self.get_logger().debug(f"Unsync detected")
            # Sync with header
            self.input_buf = self.input_buf[start:]
            # Input buffer >= 32, try parse data
            if len(self.input_buf) >= 32:
                # Move input buffer packet to local buffer
                local_buf = self.input_buf[:32]
                self.input_buf = self.input_buf[32:]
                data = list(local_buf)
                # Parse data
                success = self.parseData(data)
                # Clear timeout timer and input buffer if receive successfully
                if(success):
                    self.timeout_timer.reset()
                    self.input_buf = b''
                    
        if(DUMMY_RECEIVER):
            output_data = [1500] * 18
            for i in range(18):
                output_data[i] += random.randrange(-400, 400)
            self.get_logger().debug(f"output data: {output_data}")
            self.channel_data = output_data
            linear_vel = (output_data[1] - 1500) * 0.001
            ang_vel = -(output_data[0] - 1500) * 0.001
            self.twist.angular.z = ang_vel
            self.twist.linear.z = linear_vel
                

    def timeoutTimerCallback(self):
        self.get_logger().warning(f"Timeout detected")
        self.channel_data = [1000] * 18
        self.wheel_control = [0.0, 0.0]
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        
    def loopbackCallback(self, data: ReceiverData):
        self.get_logger().info(f'loopback channel_data: {data.channel_data}')
        self.get_logger().info(f'loopback wheel_control: {data.wheel_control}')
        self.get_logger().info(f'loopback twist: {data.twist}')
        

    def publishTimerCallback(self):
        receiver_data = ReceiverData()
        receiver_data.channel_data = self.channel_data
        if self.signal_alart==1:
            self.get_logger().info("Go")
            self.pub_raw.publish(receiver_data)
            self.pub_twist.publish(self.twist)
            self.get_logger().info(f"twist(m/s) linear: {self.twist.linear.z}, angular: {self.twist.angular.z}")
        elif self.signal_alart == 0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().warn(f"Twist: linear={self.twist.linear}, angular={self.twist.angular}")

            self.pub_twist.publish(self.twist)
            self.get_logger().error("Stop")





def main(args=None):
    rclpy.init(args=args)
    rc_receiver = RC_Receiver()
    rclpy.spin(rc_receiver)
    rc_receiver.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()