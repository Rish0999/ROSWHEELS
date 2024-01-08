#!/usr/bin/env python
import serial
import time
import rospy
from geometry_msgs.msg import Twist

# Adjust the serial port and baud rate according to your setup
serial_port = '/dev/ttyACM1'
baud_rate = 9600

# Establish the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)


def callback(data):
    # rospy.loginfo(data.linear.x)
    try:
        while True:
            
            # Get an integer from the user (you can replace this with any source of integers)
            value_to_send = [int(data.linear.x), int(data.linear.y), int(data.linear.z), int(data.angular.x), int(data.angular.y), int(data.angular.z)]
            # Send the integer to the Arduino
            # value_to_send=str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.linear.z) + " " + str(data.angular.x) +  " " + str(data.angular.y) + " " + str(data.angular.z)
            print(value_to_send)
            # ser.write(value_to_send.encode())
            # ser.write(value_to_send.encode())
            ser.write(bytes(value_to_send))
            # Optionally, wait for a short time to avoid overwhelming the Arduino
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting program.")
        ser.close()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
