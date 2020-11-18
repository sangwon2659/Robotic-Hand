#!/usr/bin/env python
import rospy
import getch
from msgpkg.msg import angle_finger2

def talker():
	pub = rospy.Publisher('jointpos', angle_finger2, queue_size=10)
	rospy.init_node('GripperAngle_Send', anonymous = True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		x = input("Angle 1: ")
		y = input("Angle 2: ")
		t = angle_finger2()
		t.Data1 = x
		t.Data2 = y
		pub.publish(t)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

