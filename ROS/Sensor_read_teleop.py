#!/usr/bin/env python
import rospy
import serial
import struct
import numpy as np
import threading
from msgpkg.msg import magnetic_encoder
from msgpkg.msg import tactile
from msgpkg.msg import msgtor

n_ch = 6
data = np.array((0,0,0,0,0,0))
value = data
alpha = 0.0
torque = 0
iteration = 0
torque_sum = 0
encoder_baseline = 0

ser = serial.Serial(port = '/dev/ttyUSB4',baudrate=115200)

def thread_run():
	global value, pub, pub2, torque
	pub.publish(value)
        pub2.publish(torque)
	rospy.loginfo(value)
	rospy.loginfo(torque)

	threading.Timer(0.01, thread_run).start()

def talker():
	global value, pub, pub2, torque, iteration, torque_sum, encoder_baseline

	pub = rospy.Publisher('tact', tactile, queue_size=1)
        pub2 = rospy.Publisher('torque', msgtor, queue_size = 1)
	rospy.init_node('talker', anonymous=True)

	ser.reset_input_buffer()
	thread_run()
	
	while not rospy.is_shutdown():
		response = ser.readline()
		if response.__len__() == n_ch*2+1:
			data = np.array(struct.unpack('<HHHHHH', response[:-1]))
			value = (alpha*value+(1-alpha)*data).astype('int32')
			iteration += 1
	        if iteration <= 500:
	        	torque_sum += value[-1]
			torque = 'N/A'
		elif iteration == 501:
			encoder_baseline = torque_sum/(iteration-1)
			torque = 'N/A'	        
		else:
			torque = abs(encoder_baseline-value[-1])*(0.2997/1504)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


