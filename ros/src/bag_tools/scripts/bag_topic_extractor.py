#!/usr/bin/env python

import subprocess
import platform
import os
import sys
import csv
import pdb
import rospy
import rosbag
from std_msgs.msg import Float32

def doExtractPlayerEyeTracking(bag, bag_path, topic, out_folder):
   bag_start_time = rospy.Time(bag.get_start_time())
   bag_name = os.path.basename(bag_path)
   topic_file_suffix = topic.replace('/','_')
   out_file_path = out_folder+'/'+bag_name[:-4]+topic_file_suffix+'.csv'
   with open(out_file_path, 'wb') as csvfile:
      csv_writer = csv.writer(csvfile, delimiter=',')
      csv_writer.writerow(['Time(sec)','PosX(norm)','PosY(norm)'])
      for bag_topic, msg, t in bag.read_messages():
         if bag_topic == topic:
            csv_writer.writerow([(t-bag_start_time).to_sec(),msg.posX, msg.posY])
   return

def doExtractAudio(bag, bag_path, topic, out_folder):
   bag_name = os.path.basename(bag_path)
   topic_file_suffix = topic.replace('/','_')
   out_file_path = out_folder+'/'+bag_name[:-4]+topic_file_suffix+'.mp3'
   mp3_file = open(out_file_path, 'w')
   for bag_topic, msg, t in bag.read_messages():
      if bag_topic == topic:
         mp3_file.write(''.join(msg.data))
   mp3_file.close()
   return

def doExtractCompressedFrames(bag, bag_path, topic, out_folder):
   frames_dir = '/tmp/temp_frames'
   if os.path.isdir(frames_dir):
      os.system('rm -rf '+frames_dir)
   os.mkdir(frames_dir)

   frame_count = 0
   for bag_topic, msg, t in bag.read_messages():
      if bag_topic == topic:
         image_format = msg.format
         if 'jpeg' in image_format:
            image_format = 'jpg'

         out_frame = open(frames_dir+'/frame%04d.'%(frame_count)+image_format, 'wb')
         out_frame.write(bytearray(msg.data))
         out_frame.close()
         frame_count = frame_count + 1

   frame_rate = round(frame_count/(bag.get_end_time()-bag.get_start_time()))

   # Generate a video from the frames
   bag_name = os.path.basename(bag_path)
   topic_file_suffix = topic.replace('/','_')
   out_file = out_folder+'/'+bag_name[:-4]+topic_file_suffix+'.mp4'
   if os.path.isfile(out_file):
      os.remove(out_file)
   os.system('cd '+frames_dir+'; ffmpeg -framerate '+str(frame_rate)+' -i frame%04d.jpg -c:v libx264 -r '+str(frame_rate)+' -pix_fmt yuv420p '+out_file)
   return

def doExtractTopic(bag_path, topic, out_folder):
   bag = rosbag.Bag(bag_path, 'r')
   type_topics = bag.get_type_and_topic_info()
   topics = type_topics[1].keys()
   types = []
   for i in range(len(type_topics[1].values())):
      types.append(type_topics[1].values()[i][0])

   topic_indices = []
   try:
      for i in range(len(topics)):
         bag_topic = topics[i]
         if bag_topic.startswith(topic):
            topic_indices.append(i)
   except ValueError:
      print 'Topic not found in the source bag.  Please check the name and try again.'
      return

   msg_types = []
   for topic_index in topic_indices:
      msg_types.append(types[topic_index])

   for i in range(len(topic_indices)):
      bag_topic = topics[topic_indices[i]]
      msg_type = msg_types[i]
      if msg_type == 'sensor_msgs/CompressedImage':
         doExtractCompressedFrames(bag, bag_path, bag_topic, out_folder)
      elif msg_type == 'audio_common_msgs/AudioData':
         doExtractAudio(bag, bag_path, bag_topic, out_folder)
      elif msg_type == 'player_eye_tracking/Position2D':
         doExtractPlayerEyeTracking(bag, bag_path, bag_topic, out_folder)

   bag.close()
   return
   
                     
if __name__=='__main__':
   rospy.init_node('bagTopicExtractor')

   if len(sys.argv) < 4:
      print 'Please provide the following command line args:\n bag_topic_extractor.py [bag_name] [topic] [out_folder]'
   else:
      bag_name = sys.argv[1]
      topic = sys.argv[2]
      out_folder = sys.argv[3]

      doExtractTopic(bag_name, topic, out_folder)
