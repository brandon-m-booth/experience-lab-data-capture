#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool, String

def callbackFunc(data):
   #rospy.loginfo("Received annotation: "+str(data.data))
   pass # Do nothing for now...

def doListenAnnotations():
   # TODO - Make the list of subscriptions driven by a data file
   #        rather than hard-coded!
   rospy.init_node('annotationListener')
   subEngagement = rospy.Subscriber("annotation/engagement", Float64, callbackFunc)
   subIsTakingNotes = rospy.Subscriber("annotation/isTakingNotes", Bool, callbackFunc)
   subSessioncode = rospy.Subscriber("annotation/sessionCode", String, callbackFunc)
   while not rospy.is_shutdown():
      rospy.spin()

if __name__=='__main__':
   try:
      doListenAnnotations()
   except rospy.ROSInterruptException:
      pass
