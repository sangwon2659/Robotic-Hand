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
import math
import time
from dynamixel_sdk import *
from msgpkg.msg import finger
from msgpkg.msg import realVal
M_PI = 3.14159265358979323846
mag_en = 0.0
t_pre = 0.0
hz = 1./200
dxl_present_position=0

k_sc = 0.55
l_length = 0.068
# Control table address
ADDR_PRO_RETURN_TIME	    = 9
ADDR_PRO_OPERATING_MODE	    = 11
ADDR_PRO_HOMING_OFFSET 	    = 20
ADDR_PRO_MAX_POS 	    = 48
ADDR_PRO_MIN_POS 	    = 52
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 4               # Dynamixel#1 ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB3'    # Check which port is being used on your controller/tact/Data[6]
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# ** setserial /dev/ttyUSB0 low_latency - if you want more communication speed, write this in terminal   https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/80


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def callback(msgmag2): 
    global mag_en   
    mag_en = msgmag2.mag2

def callback_realVal(realVal):
    global q_d
    q_d_offset = 11965
    q_d = (realVal.adc1 - q_d_offset)/(16384.0)*2.0*M_PI

def dxl_pos_read():    
    global t_pre
    global dxl_present_position
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)
    rospy.init_node('list/tact/Data[6]ener', anonymous=True)
    dxl_goal_position = 2048
    #dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    init_pos = dxl_present_position

    t_pre = time.time()
    pub = rospy.Publisher('angle_dxl', finger, queue_size=1)
    pub2 = rospy.Publisher('angle_dxl_des', finger, queue_size=1)
    rospy.Subscriber("position", realVal, callback_realVal, queue_size = 1)
    #pub3 = rospy.Publisher('force_finger_des', Float32, queue_size=1)
    #pub4 = rospy.Publisher('angle_dxl', Float32, queue_size=1)
    #pub5 = rospy.Publisher('angle_defl', Float32, queue_size=1)
    cnt = 0.0
    t0 = time.time()
    while not rospy.is_shutdown():
	cur = time.time()
	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	if (cur - t_pre)>hz:
	    t_cur = cur-t0;
	    
	    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

	    # Ramp Input
	    ''' 
	    max_input = -0.8 #Change from 0.18 to 0.21 due to change in figure (round -> rectangle)
	    start_time = 5
	    duration = 30./2.
	    itertaion = 4

	    if t_cur < start_time:
		theta_des = 0
	    elif t_cur < start_time + duration:
		theta_des = (t_cur-start_time)/duration*max_input
	    elif t_cur < start_time + duration*2.:
		theta_des = (1.-(t_cur-start_time - duration)/duration)*max_input
	    elif t_cur < start_time + duration*3.:
		theta_des = (t_cur - start_time - duration*2)/duration*max_input
	    elif t_cur < start_time + duration*4.:
		theta_des = (1.-(t_cur - start_time - duration*3)/duration)*max_input
	    elif t_cur < start_time + duration*5.:
		theta_des = (t_cur - start_time - duration*4)/duration*max_input
	    elif t_cur < start_time + duration*6.:
		theta_des = (1.-(t_cur - start_time - duration*5)/duration)*max_input
#	    elif t_cur < start_time + duration*7.:
#		theta_des = (t_cur-185.)/30.*max_input
#	    elif t_cur < start_time + duration*8.:
#		theta_des = (1.-(t_cur-215.)/30.)*max_input
	    else:
		theta_des = 0
	    #print(t_cur, theta_des, dxl_present_position)
	    #print("t_cur: {0:6f}, theta_des: {1:8f}, dxl_present_pos: {2}".format(t_cur, theta_des, dxl_present_position))
	    

	    # Sinusoidal Input
	    
	    mag = 0.2
	    if t_cur<5 :
		theta_des = 0
	    elif t_cur< 125:
		theta_des = -mag*math.sin(M_PI*(t_cur-5)/120)
		if theta_des >0:
		    theta_des = 0
		    #theta_des = -0.4*math.sin(M_PI*(t_cur-5)/2)
		    #theta_des *= -0.5
		#theta_des = 0.00
	    else :
		theta_des = 0
	    #print(t_cur,theta_des, dxl_present_position)
	    
	    
	     
	    # Step Input/tact/Data[6]
	    
	    max_input = 0.05
	    if t_cur<5:
		theta_des = 0
	    elif t_cur< 20:
		theta_des = -max_input
	    elif t_cur< 40:
		theta_des = -max_input*2
	    elif t_cur< 60:
		theta_des = -max_input
	    elif t_cur< 80:
		theta_des = -max_input*2
	    elif t_cur< 100:
		theta_des = -max_input
	    else :
		theta_des = 0
#	    print(t_cur,theta_des, dxl_present_position)
	     
	    
	    # Ramp Step Input
	    if t_cur < 5:
		theta_des = 0
	    elif t_cur < 20:
		theta_des = (t_cur - 5)/15*-0.2
	    elif t_cur < 100:
		theta_des = -0.2
	    elif t_cur < 115:
		theta_des = (1-(t_cur-100)/15)*-0.2
	    else:
		theta_dse = 0
	    ''' 
            
            if q_d < -0.25:
	    	theta_des = -0.25
 	    elif q_d > 0.25:
	    	theta_des = 0.25
	    else:
	    	theta_des = q_d
	    theta_cur = float(dxl_present_position-init_pos)/2048*M_PI
	    dxl_goal_position = int(theta_des/M_PI*2048+init_pos)
	    print('T: %3.3f \t Goal: %4d \t Pre: %4d \t Err: %2d \t ang_d: %2.3f'
	%(round(t_cur,3),dxl_goal_position, dxl_present_position, dxl_goal_position-dxl_present_position, theta_des))
	    pub.publish(theta_cur)
	    pub2.publish(theta_des)
	 
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            #dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	    t_pre = cur

	    if(abs(theta_des)>1.3):
		dxl_goal_position = 2048
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
		break


if __name__ == '__main__':
    try:
        t_pre = time.time()
        dxl_pos_read()
    except rospy.ROSInterruptException:
        pass
