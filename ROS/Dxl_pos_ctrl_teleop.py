#!/usr/bin/env python

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
portHandler = PortHandler(DEVICENAME)
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
    cnt = 0.0
    t0 = time.time()
    while not rospy.is_shutdown():
	cur = time.time()
	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	if (cur - t_pre)>hz:
	    t_cur = cur-t0;
	    
	    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
            
            # Directly puts in the desired angle info that is received from the <realVal> subscriber
	    # Setting boundaries for the angle to prevent damage
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
