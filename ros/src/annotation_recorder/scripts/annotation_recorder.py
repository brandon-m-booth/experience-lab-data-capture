#!/usr/bin/env python

import pdb
import rospy
import rosbag
from std_msgs.msg import Float64, Bool, String

ignoreMessages = False
startTime = None
lastTime = rospy.Time(0)
topic_dict_time_series = {}

def callbackFunc(data, topic=None):
   global ignoreMessages
   global startTime
   global lastTime
   global topic_dict_time_series

   if ignoreMessages:
      return

   now = rospy.Time.now()
   if now.to_sec() <= 0.0:
      return
   elif startTime is None:
      startTime = now

   if topic is None:
      rospy.logerror("Callback function's topic is None, FIX ME!!")

   if now < lastTime:
      if lastTime > (startTime + rospy.Duration(20*60)) and now < (startTime + rospy.Duration(20)): # If the bag has looped back to the beginning
         ignoreMessages = True
         return

      for dict_topic in topic_dict_time_series.keys():
         clip_index = next((x[0] for x in enumerate(topic_dict_time_series[dict_topic]) if x[1][2] >= now), None)
         if not clip_index is None:
            topic_dict_time_series[dict_topic] = topic_dict_time_series[dict_topic][0:clip_index]
   lastTime = now
   topic_dict_time_series[topic].append((topic, data, now))

   return

###########################################################################

def writeAnnotationsBag():
   global topic_dict_time_series

   rospy.loginfo("Writing annotations to bag file, please wait...")
   with rosbag.Bag('/home/elab/Desktop/outbag.bag', 'w') as outbag:
      for dict_topic in topic_dict_time_series.keys():
         for (topic, msg, t) in topic_dict_time_series[dict_topic]:
            outbag.write(topic, msg, t)

###########################################################################

def doRecordAnnotations():
   global topic_dict_time_series

   # TODO - Make the list of subscriptions driven by a data file
   #        rather than hard-coded!
   rospy.init_node('annotationRecorder')
   rospy.on_shutdown(writeAnnotationsBag)

   topics_and_msgtypes = [("annotation/engagement",Float64), ("annotation/isTakingNotes", Bool), ("annotation/sessionCode", String)]
   for (topic, msg_type) in topics_and_msgtypes:
      sub = rospy.Subscriber(topic, msg_type, lambda data,tp=topic: callbackFunc(data, topic=tp))
      topic_dict_time_series[topic] = []

   while not rospy.is_shutdown():
      rospy.spin()


if __name__=='__main__':
   doRecordAnnotations()
