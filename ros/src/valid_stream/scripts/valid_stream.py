#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def doValidStreamPublish():
   pub = rospy.Publisher('valid_stream', Bool, queue_size=1)
   rospy.init_node('validStreamNode')
   rospy.set_param('/valid_stream', True)
   rate = rospy.Rate(2) # Repeat every X Hertz
   while not rospy.is_shutdown():
      is_stream_valid = rospy.get_param('/valid_stream')
      validStreamMsg = Bool()
      validStreamMsg.data = is_stream_valid
      pub.publish(validStreamMsg)
      rate.sleep()

if __name__=='__main__':
   try:
      doValidStreamPublish()
   except rospy.ROSInterruptException:
      pass
