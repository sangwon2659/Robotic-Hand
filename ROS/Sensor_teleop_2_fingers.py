#!/usr/bin/env python
import rospy
import serial
import struct
import numpy as np
import threading
from msgpkg.msg import tactile
from msgpkg.msg import msgtor2

# 5 channels including one encoder as the last slot
n_ch = 5
# Declaring the data array
data = np.array((0,0,0,0,0))
value = data
alpha = 0.0
torque = 0

data2 = np.array((0,0,0,0,0))
value2 = data2
alpha2 = 0.0
torque2 = 0

# Number of iteration until encoder offset determined
iteration = 0
torque_sum = 0
# Offset for measurements of encoder
encoder_baseline = 0

torque_sum2 = 0
encoder_baseline2 = 0

# Port and baudrate declaration
ser = serial.Serial(port = '/dev/ttyUSB4',baudrate=115200)
ser2  = serial.Serial(port = '/dev/ttyUSB3',baudrate=115200)

def thread_run():
	global value, pub, pub2, torque
	# Publishing and posting the value array(4 tact info and 1 uncalibrated angle info) and the torque
	#pub.publish(value)
	#pub.publish(value2)	
	t = msgtor2()
	t.tor1 = torque
	t.tor2 = torque2
        pub2.publish(t)
	rospy.loginfo(value)

	threading.Timer(0.01, thread_run).start()

def talker():
	global value, pub, pub2, torque, iteration, torque_sum, encoder_baseline
	# Declaring publish variables
	# Msg <tactile> for the value array
	# Msg <msgtor2> for the torque
	pub = rospy.Publisher('tact', tactile, queue_size=1)
        pub2 = rospy.Publisher('teletorque', msgtor2, queue_size = 1)
	# Initializing node
	rospy.init_node('talker', anonymous=True)

	ser.reset_input_buffer()
	thread_run()
	
	while not rospy.is_shutdown():
		# Reading data from port
		response = ser.readline()
		if response.__len__() == n_ch*2+1:
			# Putting in the 6 values into data
			# Last byte not taken
			data = np.array(struct.unpack('<llllll', response[:-1]))
			value = (alpha*value+(1-alpha)*data).astype('int32')

		# Reading data from port
		response2 = ser2.readline()
		if response2.__len__() == n_ch*2+1:
			# Putting in the 6 values into data
			# Last byte not taken
			data2 = np.array(struct.unpack('<llllll', response2[:-1]))
			value2 = (alpha*value2+(1-alpha)*data2).astype('int32')

		iteration += 1

		# Finding the offset for the encoder
		# Displayed as 'N/A' before the number of iterations is met
		# Calibrates the angle info using the pre-done calibration and the current offset
	        if iteration <= 500:
	        	torque_sum += value[-1]
			torque = 'N/A'
	        	torque_sum2 += value2[-1]
			torque2 = 'N/A'
		elif iteration == 501:
			encoder_baseline = torque_sum/(iteration-1)
			torque = 'N/A'	        
			encoder_baseline2 = torque_sum2/(iteration-1)
			torque2 = 'N/A'	        
		else:
			torque = abs(encoder_baseline-value[-1])*(0.000002893)
			torque2 = abs(encoder_baseline2-value2[-1])*(0.000002893)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
