#!/usr/bin/env python3

import struct
import serial
import rclpy
from rclpy.node import Node
from krabi_msgs.msg import Encoders, MotorsCmd, MotorsParameters, OdomLighter
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class SimpleBrokerSTM32(Node):
    def __init__(self):
        super().__init__('simple_broker_stm32')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback_cmd_vel,
            10)
        self.subscription = self.create_subscription(
            MotorsCmd,
            'motors_cmd',
            self.callback_motors_cmd,
            10)
        self.subscription = self.create_subscription(
            MotorsParameters,
            'motors_parameters',
            self.callback_motors_parameters,
            10)
        self.subscription = self.create_subscription(
            Bool,
            'enable_motors',
            self.callback_enable_motors,
            10)
        self.odom_pub = self.create_publisher(OdomLighter, 'odom_lighter', 10)
        self.encoders_pub = self.create_publisher(Encoders, 'encoders', 10)
        self.timer = self.create_timer(0.1, self.read_serial)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

    def callback_enable_motors(self, msg):
        float_array_to_send = []
        float_array_to_send.append(msg.data)
        
        self.write_float_to_serial("e", float_array_to_send)

    def callback_motors_parameters(self, msg):
        float_array_to_send = []
        float_array_to_send.append(msg.max_current_left)
        float_array_to_send.append(msg.max_current_right)
        float_array_to_send.append(msg.max_current)
        
        self.write_float_to_serial("p", float_array_to_send)

    def callback_motors_cmd(self, msg):
        float_array_to_send = []
        float_array_to_send.append(msg.enable_motors)
        float_array_to_send.append(msg.override_pwm)
        float_array_to_send.append(msg.pwm_override_left)
        float_array_to_send.append(msg.pwm_override_right)
        float_array_to_send.append(msg.reset_encoders)
        
        self.write_float_to_serial("c", float_array_to_send)

    def callback_cmd_vel(self, msg):
        float_array_to_send = []
        float_array_to_send.append(msg.linear.x)
        float_array_to_send.append(msg.angular.z)
        
        self.write_float_to_serial("v", float_array_to_send)

    # TODO implement other writers, then readers

    def float_to_hex(self, float_value):
        # Convert float to hexadecimal representation
        float_bytes = struct.pack('f', float_value)
        float_hex = ''.join(format(x, '02x') for x in float_bytes)
        return float_hex
        

    def write_float_to_serial(self, channel_name, float_values):
        # Convert list of float values to hexadecimal representation and concatenate them
        # [2:] to remove "0x"
        # zfill(8) so each float takes up 8 char
        # struct pack/unpack to convert float to int
        hex_values = "".join([hex(struct.unpack('I', struct.pack('f', float(float_value)))[0])[2:].zfill(8) for float_value in float_values])
        # Send concatenated hexadecimal values over serial
        self.ser.write(f"{channel_name}{hex_values}\n".encode())
        #print(f"sending:{hex_values}\n".encode())
        self.get_logger().debug(f"Sending: {hex_values}\n".encode())

    def read_serial(self):
        line = self.ser.readline().decode().strip()
        self.get_logger().debug(f"Line: {line}")

        if line.startswith("o:"):
            odom_msg = OdomLighter()
            hex_value = line.split(":")[-1]
            float_hex = int(hex_value, 16)
            odom_msg.pose_x, odom_msg.pose_y, odom_msg.angle_rz, odom_msg.speed_vx, odom_msg.speed_wz = struct.unpack('f', struct.pack('I', float_hex))
            self.get_logger().debug(f"Received Odom: {odom_msg}")
            
            self.odom_pub.publish(odom_msg)

        if line.startswith("e:"):
            encoders_msg = Encoders()
            hex_value = line.split(":")[-1]
            float_hex = int(hex_value, 16)
            encoders_msg.encoder_right, encoders_msg.encoder_left = struct.unpack('f', struct.pack('I', float_hex))
            self.get_logger().debug(f"Received Encoders: {encoders_msg}")
            
            self.encoders_pub.publish(encoders_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_broker_STM32 = SimpleBrokerSTM32()
    try:
        rclpy.spin(simple_broker_STM32)
    finally:
        simple_broker_STM32.ser.close()
        simple_broker_STM32.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()