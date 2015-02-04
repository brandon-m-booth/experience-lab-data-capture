#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sayHello():
	pub = rospy.Publisher('hello', String, queue_size=1)
	rospy.init_node('helloNode', anonymous=True)
	rate = rospy.Rate(1) # Repeat every Hertz
	while not rospy.is_shutdown():
		helloString = "Hello World!\n"
		rospy.loginfo("Publishing string: %s" % helloString)
		pub.publish(helloString)
		rate.sleep()

if __name__=='__main__':
	try:
		sayHello()
	except rospy.ROSInterruptException:
		pass
