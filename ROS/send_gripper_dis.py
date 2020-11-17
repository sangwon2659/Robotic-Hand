#!/usr/bin/env python
import rospy
import getch
from msgpkg.msg import gripper_dis

def talker():
	pub = rospy.Publisher('GripperDis', gripper_dis, queue_size=10)
	rospy.init_node('GripperDis_Send', anonymous = True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		x = input("Gripper Distance: ")
		rospy.loginfo(str(x))
		pub.publish(x)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass


