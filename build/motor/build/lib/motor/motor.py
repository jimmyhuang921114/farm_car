import os
import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from motor_if.srv import MotorPos
import math as m

motor_obj = None 


class UsbDevice:
    def __init__(self):
        pass

    def usb_init(self, usb='/dev/ttyUSB0', baudrate=115200, protocol_version=2.0):
        self.usb = usb
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.portHandler = PortHandler(self.usb)
        self.packetHandler = PacketHandler(self.protocol_version)
        self.set_baudrate()
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

    def set_baudrate(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

    def close(self):
        self.portHandler.closePort()




class Motor2Wheel(UsbDevice):
    def __init__(self):
        pass

    def motor_init(self, m1_id=1, m2_id=2):
        # Save ID
        self.m1_id = m1_id
        self.m2_id = m2_id
        # Set Position Limit
        # DXL_MINIMUM_POSITION_VALUE  = 0   # Refer to the Minimum Position Limit of product eManual
        # DXL_MAXIMUM_POSITION_VALUE  = 4095    # Refer to the Maximum Position Limit of product eManual
        # dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
        # Motor Init
        # m1_comm_result, m1_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m1_id, 48, dxl_goal_position[0])
        # m2_comm_result, m2_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m2_id, 48, dxl_goal_position[0])
        # Set Velocity mode
        print("Write Velocity Mode")
        comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 11, 1)
        self.print_error(comm_result, error)
        comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 11, 1)
        self.print_error(comm_result, error)
        # Torque Enable
        print("Write Torque Enable")
        comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 562, 1)
        self.print_error(comm_result, error)
        comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 562, 1)
        self.print_error(comm_result, error)

    def ping(self):
        m1_model_number, m1_comm_result, m1_error = self.packetHandler.ping(self.portHandler, self.m1_id)
        if not self.print_error(m1_comm_result, m1_error):
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m1_id, m1_model_number))
        m2_model_number, m2_comm_result, m2_error = self.packetHandler.ping(self.portHandler, self.m2_id)
        if not self.print_error(m2_comm_result, m2_error):
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m2_id, m2_model_number))

    def print_error(self, comm_result, error) -> bool:
        ''' 
            return True when failed 
        '''
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(comm_result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
        else:
            return False
        return True

    def set_speed(self, m1_speed, m2_speed):
        '''
            speed: rpm
        '''
        ## factor 0.229 from XM430-W350 datasheet
        # Motor 1
        comm_result, error = self.packetHandler.write4ByteTxRx(self.portHandler,self.m1_id, 600, int(m1_speed / 0.00199234))
        self.print_error(comm_result, error)
        # Motor 2
        comm_result, error = self.packetHandler.write4ByteTxRx(self.portHandler,self.m2_id, 600, int(m2_speed / 0.00199234))
        self.print_error(comm_result, error)



class MotorNode(Node):
    def __init__(self):
        global motor_obj
        super().__init__('motor_node')
        self.sub_motor_main = self.create_subscription(
            Twist,
            '/motor/main',
            self.receive_motor_main_callback,
            1)
        self.sub_motor_aux = self.create_subscription(
            Twist,
            '/motor/aux',
            self.receive_motor_aux_callback,
            1)
        self.srv_motor_pos = self.create_service(
            MotorPos, 
            '/motor/pos', 
            self.motor_pos_callback)
        ''' Car Struct '''
        self.wheel_radius = 0.105 # meters
        self.wheel_dist = 0.75 # meters
        ''' Variable '''
        self.main_linear = 0
        self.main_angular = 0
        self.aux_angular = 0
        ''' Motor '''
        self.motor = Motor2Wheel()
        motor_obj = self.motor
        self.motor.usb_init(usb='/dev/ttyUSB0')
        self.motor.motor_init(m1_id=2, m2_id=1)
        self.motor.ping()
        self.motor.set_speed(0, 0)
        time.sleep(3)
        self.get_logger().info("init finish")
        ''' debug '''


    def receive_motor_main_callback(self, msg: Twist):
        linear = msg.linear.z # m/s
        angular = msg.angular.z # rad/s
        self.get_logger().info(f"twist main: linear-> {linear}, angular-> {angular}")
        self.main_linear = linear
        self.main_angular = angular
        self.set_target()


    def receive_motor_aux_callback(self, msg: Twist):
        angular = msg.angular.z # rad/s
        self.get_logger().info(f"twist aux: angular-> {angular}")
        self.aux_angular = angular
        self.set_target()


    def motor_pos_callback(self, request, response):
        REF_LINEAR = 0.1 # m/s
        REF_ANGULAR = 1 # rad/s
        linear = request.linear # m/s
        angular = request.angular # rad/s
        self.get_logger().info(f"twist pos: linear-> {linear}, angular-> {angular}")
        if linear and angular:
            self.get_logger().warn(f"twist pos input linear and angular at same time")
        self.aux_angular = self.main_angular = self.main_linear = 0
        if angular:
            dt = angular / REF_ANGULAR
            if dt >= 0:
                self.main_angular = REF_ANGULAR
            else:
                self.main_angular = -REF_ANGULAR
            self.set_target()
            time.sleep(abs(dt))
            self.main_angular = 0
            self.set_target()
        elif linear:
            dt = linear / REF_LINEAR
            if dt >= 0:
                self.main_linear = REF_LINEAR
            else:
                self.main_linear = -REF_LINEAR
            self.set_target()
            time.sleep(abs(dt))
            self.main_linear = 0
            self.set_target()
        ## pub status
        response.result = 1
        return response


    def set_target(self):
        linear = self.main_linear # m/s
        angular = self.main_angular + self.aux_angular # rad/s
        self.get_logger().info(f"twist target: linear-> {linear}, angular-> {angular}")
        speed_left = linear - (self.wheel_dist / 2) * angular
        speed_right = linear + (self.wheel_dist / 2) * angular
        output_left = 60 * speed_left / (self.wheel_radius * 2 * m.pi)     # mult 60 for rps to rpm
        output_right = -60 * speed_right / (self.wheel_radius * 2 * m.pi)   # mult 60 for rps to rpm
        self.get_logger().info(f"output target: left: {output_left}, right: {output_right}")
        self.motor.set_speed(output_left, output_right)


def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    try:
        rclpy.spin(motor_node)
    except:
        motor_obj.set_speed(0, 0)
    rclpy.shutdown()




if __name__ == '__main__':
    motor = Motor2Wheel()
    motor.usb_initialization(usb='/dev/ttyACM0')
    motor.motor_initialization(m1_id=1, m2_id=2)
    motor.ping()
    motor.setSpeed(0, 0)