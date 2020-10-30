#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
import time
import serial
from std_msgs.msg import Float64
from msgpkg.msg import msgtor
from msgpkg.msg import tactile

ser = serial.Serial(port = '/dev/ttyUSB3',baudrate=115200)

def talker():
    #pub = rospy.Publisher('torque', msgtor, queue_size=5)
    pub = rospy.Publisher('FT', tactile, queue_size=5)
    rospy.init_node('Mini45', anonymous=True)
    #rate = rospy.Rate(1000) # 10hz
    res = np.array([1./2., 1./2., 1./2., 1./94., 1./94., 1./188.])/.8

    while not rospy.is_shutdown():
        ser.write(chr(20))
        #ser.write('^T')  # Communication for Resolved force/torque
        response = ser.readline()
	response = response[:-2].split(',')
	response = map(float, response)
	response = np.array(response[1:])
	print(response*res)
	pub.publish(response)
        #where = response.find(',')
        #va = float(response[where+1:])/188/8 #/2/8  #/188/8
	#value = float(ser.readline())
        #rospy.loginfo(va)
        #pub.publish(va)
        #rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
