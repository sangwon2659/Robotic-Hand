#!/usr/bin/env python
import rospy
import numpy as np
import pickle
from std_msgs.msg import Int32
from msgpkg.msg import tactile

#Loading the model (filename should be in full)
filename = '/home/a283/roboticfinger_ros/src/roboticfinger/script/SVM_linear_kernel.sav'
loaded_model = pickle.load(open(filename,'rb'))

#n_ch refers to the length of data - 1 (the encoder data)
n_ch = 4
buffer_ = np.zeros((n_ch,1))
n = 0

def callback(data):
	global buffer_
	data = np.asarray(data.Data[0:n_ch])
	buffer_ = np.append(data,buffer_)
	buffer_ = np.delete(buffer_, 6, 0)
	rospy.loginfo(buffer_)
	#if n > 4:
	#	offset_average = 	
	#	for i in range(n_ch):
	#		result[i][0] = loaded_model.predict(data.Data[i])

def talklistener():
	rospy.init_node('touch', anonymous=True)	
	rospy.Subscriber("tact", tactile, callback)
	rospy.spin()
	#pub = rospy.Publisher('touch', Int32, queue_size=10)

if __name__ == '__main__':
	try:
		talklistener()
	except rospy.ROSInterruptException:
		pass
		




