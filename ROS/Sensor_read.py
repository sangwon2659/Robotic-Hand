#!/usr/bin/env python
import rospy
import serial
import struct
import numpy as np
import threading
from msgpkg.msg import magnetic_encoder
from msgpkg.msg import tactile

n_ch = 3
# Declaring an array so store data and display it later
data = np.array((0,0,0))
value = data
alpha = 0.0

# Declaring the USB port for which the arduino is connected
ser = serial.Serial(port = '/dev/ttyUSB3',baudrate=115200)

def thread_run():
	global value, pub
	# Publishing the value
	pub.publish(value)
	# Displaying the value on the log
	rospy.loginfo(value)

	threading.Timer(0.01, thread_run).start()

def talker():
	global value, pub

	pub = rospy.Publisher('tact', tactile, queue_size=1)
	rospy.init_node('talker', anonymous=True)

	ser.reset_input_buffer()
	thread_run()

	while not rospy.is_shutdown():
		# Reading the data from the arduino
		response = ser.readline()
		# 2 bytes of data for each channel from the arduino + 1 byte of spacing
		if response.__len__() == n_ch*2+1:
			# A method of unpacking the data
			# There are 3 Hs since the number of channels is 3
			data = np.array(struct.unpack('<HHH', response[:-1]))
			value = (alpha*value+(1-alpha)*data).astype('int32')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
