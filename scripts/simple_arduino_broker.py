#!/usr/bin/env python3

import struct
import serial
import rclpy
from rclpy.node import Node
from krabi_msgs.msg import Actuators
from std_msgs.msg import Float32

class SimpleBrokerArduino(Node):
    def __init__(self):
        super().__init__('simple_broker_arduino')
        self.subscription = self.create_subscription(
            Actuators,
            'actuators_msg',
            self.callback,
            10)
        self.vacuum_pub = self.create_publisher(Float32, 'vacuum', 10)
        self.timer = self.create_timer(0.1, self.read_serial)
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)

    def callback(self, msg):
        float_array_to_send = []
        float_array_to_send.append(msg.arm_base_servo.enable)
        
        float_array_to_send.append(msg.arm_base_servo.speed)
        float_array_to_send.append(msg.arm_base_servo.angle)
        
        float_array_to_send.append(msg.arm_mid_servo.enable)
        float_array_to_send.append(msg.arm_mid_servo.speed)
        float_array_to_send.append(msg.arm_mid_servo.angle)
        
        float_array_to_send.append(msg.arm_suction_cup_servo.enable)
        float_array_to_send.append(msg.arm_suction_cup_servo.speed)
        float_array_to_send.append(msg.arm_suction_cup_servo.angle)
        
        float_array_to_send.append(msg.pusher_servo.enable)
        float_array_to_send.append(msg.pusher_servo.speed)
        float_array_to_send.append(msg.pusher_servo.angle)
        
        float_array_to_send.append(msg.additionnal_servo_1.enable)
        float_array_to_send.append(msg.additionnal_servo_1.speed)
        float_array_to_send.append(msg.additionnal_servo_1.angle)
        
        float_array_to_send.append(msg.additionnal_servo_2.enable)
        float_array_to_send.append(msg.additionnal_servo_2.speed)
        float_array_to_send.append(msg.additionnal_servo_2.angle)
        
        float_array_to_send.append(msg.arm_vacuum.enable_pump)
        float_array_to_send.append(msg.arm_vacuum.release)
        float_array_to_send.append(msg.fake_statuette_vacuum.enable_pump)
        float_array_to_send.append(msg.score)



        self.write_float_to_serial(float_array_to_send)

    def float_to_hex(self, float_value):
        # Convert float to hexadecimal representation
        float_bytes = struct.pack('f', float_value)
        float_hex = ''.join(format(x, '02x') for x in float_bytes)
        return float_hex
        

    def write_float_to_serial(self, float_values):
        # Convert list of float values to hexadecimal representation and concatenate them
        # [2:] to remove "0x"
        # zfill(8) so each float takes up 8 char
        # struct pack/unpack to convert float to int
        hex_values = "".join([hex(struct.unpack('I', struct.pack('f', float(float_value)))[0])[2:].zfill(8) for float_value in float_values])
        # Send concatenated hexadecimal values over serial
        self.ser.write(f"{hex_values}\n".encode())
        #print(f"sending:{hex_values}\n".encode())
        #self.get_logger().debug(f"Sending: {hex_values}\n".encode())

    def read_serial(self):
        line = self.ser.readline().decode().strip()
        self.get_logger().debug(f"Line: {line}")
        if line.startswith("From Arduino:"):
            vacuum_msg = Float32()
            hex_value = line.split(":")[-1]
            float_hex = int(hex_value, 16)
            vacuum_msg.data = struct.unpack('f', struct.pack('I', float_hex))[0]
            self.get_logger().debug(f"Received Float Value: {vacuum_msg.data}")
            
            self.vacuum_pub.publish(vacuum_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_broker_arduino = SimpleBrokerArduino()
    try:
        rclpy.spin(simple_broker_arduino)
    finally:
        simple_broker_arduino.ser.close()
        simple_broker_arduino.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()