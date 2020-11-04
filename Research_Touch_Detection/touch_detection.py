#!/usr/bin/env python
'''
Receives data from FSS sensor (or the "/tact" topic)
Decides whether touching or not using the SVM model in the sav file
Publishes the result as a "/touch" topic
'''
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
buffer_ = np.zeros((n_ch,4))

def callback(data):
	global loaded_model, buffer_

	#Changing the data.Data into an array
	data = np.asarray(data.Data[0:n_ch])
	#Addending the newly received data
	buffer_ = np.append(data,buffer_)
	
	#Deleting the oldest dataset in the buffer
	buffer_ = np.delete(buffer_, (n_ch*4,n_ch*4+1,n_ch*4+2,n_ch*4+3))
	#Computing the offset
	offset_ = []
	for i in range(n_ch): 
		offset_.append(offset(i))	
	
	#Putting in the offset and lastest dataset into the model
	if buffer_[n_ch-1] != 0:
		result_ = []
		for i in range(n_ch):
			result = touch(offset(i),buffer_[i])
			result_.append(result)
		pub.publish(result_)
		rospy.loginfo(result_)

def talklistener():
	global pub
	rospy.init_node('touch', anonymous=True)	
	pub = rospy.Publisher('touch', tactile, queue_size = 10)	
	rospy.Subscriber("tact", tactile, callback)
	rospy.spin()

def offset(channel):
	sum = 0
	for i in range(1,n_ch):
		sum += buffer_[channel+n_ch*i]
	offset = sum/(n_ch-1)
	return offset

def touch(offset, data):
	prediction = np.array([[offset,data]])
	result = loaded_model.predict(prediction)
	return result

if __name__ == '__main__':
	try:
		talklistener()
	except rospy.ROSInterruptException:
		pass
		



