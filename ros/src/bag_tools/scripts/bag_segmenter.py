#!/usr/bin/env python

import pdb
import rospy
import rosbag
from std_msgs.msg import Float32

def doSegmentBags():
   rospy.init_node('bag_segmenter')

   bag_file = '/home/elab/Desktop/13_Participant_2016-05-04-20-35-37_annotated.bag'

   rospy.loginfo('Scanning session codes in bag...')
   session_time_bounds = {}
   bag = rosbag.Bag(bag_file, 'r')
   for topic, msg, t in bag.read_messages():
      if topic == "annotation/sessionCode":
         if not msg.data in session_time_bounds.keys():
            session_time_bounds[msg.data] = [t, t]
         else:
            bounds = session_time_bounds[msg.data]
            bounds[0] = min(t,bounds[0])
            bounds[1] = max(t,bounds[1])
            session_time_bounds[msg.data] = bounds
   bag.close()

   # Ooooh, so inefficient. #shame
   for session_code in session_time_bounds.keys():
      if 'none' in session_code.lower():
         continue

      rospy.loginfo('Extracting session code: %s'%session_code)
      time_bound = session_time_bounds[session_code]
      out_bag_file = bag_file[:-4]+'_'+session_code+'.bag'
      with rosbag.Bag(out_bag_file, 'w') as outbag:
         bag = rosbag.Bag(bag_file, 'r')
         for topic, msg, t in bag.read_messages():
            if t >= time_bound[0] and t <= time_bound[1]:
               outbag.write(topic, msg, t)
         bag.close()

if __name__=='__main__':
   doSegmentBags()
