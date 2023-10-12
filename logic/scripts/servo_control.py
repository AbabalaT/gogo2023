#! /usr/bin/env python3

import serial
import struct
import rospy

def servo_commmand_callback(event):
    servo_cmd = rospy.get_param('/mission/servo_pos')
    serial_buff = struct.pack('B', servo_cmd)
    serial_port.write(serial_buff)

if __name__ == "__main__":
    rospy.init_node("servo_control_node")
    serial_port = serial.Serial(
        port="/dev/ttyUSB0",   
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    timer_05 = rospy.Timer(rospy.Duration(0.1), servo_commmand_callback)
    rospy.spin()
