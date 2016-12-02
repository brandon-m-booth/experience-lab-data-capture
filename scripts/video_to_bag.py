#!/usr/bin/python

import time
import sys
import os
import pdb
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def create_video_bag(input_video_path, output_bag_path, topic):
   bag = rosbag.Bag(output_bag_path, 'w')
   cap = cv2.VideoCapture(input_video_path)
   cb = CvBridge()
   prop_fps = cap.get(cv2.CAP_PROP_FPS)
   if prop_fps != prop_fps or prop_fps <= 1e-2:
      print "Warning: can't get FPS. Assuming 24."
      prop_fps = 24
   is_valid = True
   frame_id = 0
   while(is_valid):
      is_valid, frame = cap.read()
      if not is_valid:
         break
      stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
      frame_id += 1
      image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
      image.header.stamp = stamp
      image.header.frame_id = "camera"
      bag.write(topic, image, stamp)
   cap.release()
   bag.close()

if __name__ == "__main__":
   if len(sys.argv) > 2:
      input_video = sys.argv[1]
      output_bag = sys.argv[2]
      if len(sys.argv) > 3:
         topic = sys.argv[3]
      else:
         topic = 'video/image_raw'
      create_video_bag(input_video, output_bag, topic)
   else:
      print 'Please provide the following command line arguments:\n1) Input video path\n2) Output bag path\n3) (OPTIONAL) Video topic name'
