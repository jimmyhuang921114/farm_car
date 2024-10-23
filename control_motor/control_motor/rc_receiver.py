from serial import Serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RC_Receiver(Node):
    def __init__(self):
        super().__init__('rc_receiver')
        # 直接定義串列埠
        port = "/dev/ttyUSB1"
        self.get_logger().info('Using serial port: "%s"' % port)

        # 初始化 Serial 連接
        try:
            self.serial = Serial(port, 115200)
        except Exception as e:
            self.get_logger().error(f'無法開啟序列埠，錯誤:\n\t{e}')
            exit()
        
        self.publisher_ = self.create_publisher(Twist, '/rc_receiver', 10)
        self.sub = self.create_subscription(Twist, '/rc_receiver', self.loopbackCallback, 10)
        
        self.channel_data = [1000] * 18
        self.wheel_control = [0, 0]
        
        self.timeout_timer = self.create_timer(5, self.timeoutTimerCallback)
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
        self.get_logger().debug(f"output data: {output_data}")
        self.channel_data = output_data
        linear_vel = (output_data[1] - 1500) * 0.001
        ang_vel = -(output_data[0] - 1500) * 0.001
        msg=Twist()
        msg.linear.x = linear_vel
        msg.angular.z = ang_vel

        # 發佈 Twist 訊息
        self.publisher_.publish(msg)
        return True

    def serialHandleTimerCallback(self):
        
        if self.serial.in_waiting > 0:
            self.input_buf += self.serial.read_all()
        if len(self.input_buf) >= 2:
            start = self.input_buf.find(b'\x20\x40')
            if start == -1:
                return
            if start != 0:
                self.get_logger().debug(f"Unsync detected")
            self.input_buf = self.input_buf[start:]
            if len(self.input_buf) >= 32:
                local_buf = self.input_buf[:32]
                self.input_buf = self.input_buf[32:]
                data = list(local_buf)
                success = self.parseData(data)
                if success:
                    self.timeout_timer.reset()
                    self.input_buf = b''

    def timeoutTimerCallback(self):
        self.get_logger().warning(f"Timeout detected")
        self.channel_data = [1000] * 18
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def loopbackCallback(self, msg):
        # self.get_logger().info(f'loopback: {self.channel_data}')
        self.get_logger().info(f'loopback: {msg.linear.x,msg.angular.z}')

    def publishTimerCallback(self):
        msg = Twist()
        # 根據 channel_data 設置 Twist 訊息
        linear_vel = (self.channel_data[1] - 1500) * 0.001
        ang_vel = -(self.channel_data[0] - 1500) * 0.001

        msg.linear.x = linear_vel*50
        msg.angular.z = ang_vel*50

        # 發佈 Twist 訊息
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rc_receiver = RC_Receiver()
    rclpy.spin(rc_receiver)
    rc_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
