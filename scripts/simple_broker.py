#!/usr/bin/env python

import serial
import struct
import rospy
from krabi_msgs.msg import actuators
from sts_msgs.msg import Float32

def callback(data):
    rospy.loginfo("Received actuators message: %s", data)
    

def write_float_to_serial(float_value):
    # Convert float to hexadecimal representation
    float_bytes = struct.pack('f', float_value)
    float_hex = ''.join(format(x, '02x') for x in float_bytes)
    
    # Send hexadecimal value over serial
    ser.write(f"Hex value: {float_hex}\n".encode())

def sample_node():

  rospy.init_node('simple_broker_arduino', anonymous=True)    
  rospy.Subscriber("actuators_topic", actuators, callback)
  vacuum_pub = rospy.Publisher('vacuum', Float32, queue_size=10)
  rate = rospy.Rate(50) # 50hz
  vacuum_msg = Float32()

  while not rospy.is_shutdown():
    line = ser.readline().decode().strip()
    
    # Check if the line contains hexadecimal value
    if line.startswith("Hex value:"):
        hex_value = line.split(" ")[-1]
        # Convert hexadecimal string to unsigned integer
        float_hex = int(hex_value, 16)
        # Convert unsigned integer to float
        vacuum_msg.value = struct.unpack('f', struct.pack('I', float_hex))[0]
        print("Received Float Value:", vacuum_msg.value)
        vacuum_pub.publish(vacuum_msg)
    rate.sleep()

if __name__ == "__main__":
  # Open serial port
  ser = serial.Serial('COM3', 57600, timeout=0.1)  # Adjust 'COM3' to match your serial port
  try:
    sample_node()
  finally:
    rospy.loginfo("exiting")
