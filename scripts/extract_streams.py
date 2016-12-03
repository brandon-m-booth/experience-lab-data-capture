#!/usr/bin/python

import os
import re
import pdb
import sys
import platform

def doExtractStreams(bag_path, out_path, topics=None):
   if not topics:
      topics = ['/audio', '/eye_position', '/kinect2/rgb_rect/image', '/game_video/rgb/image', '/playerInput/cursorPosition', '/playerInput/keyEvents', '/player_app_activity', '/player_eeg', '/player_video/overhead/camera/image_mono']
      for annotator_id in range(30):
         for session in ['lecture', 'student']:
            for stream in ['engagement', 'isTakingNotes']:
               topics.append('annotation/'+session+'/annotator'+str(annotator_id)+'/'+stream)
   filter_re_prog = re.compile('.*[main|main2|b1|b2]')
   bag_file_paths = []

   # Get list of all subject file paths in all subject subfolders
   subject_folders = [bag_path]
   for subject_folder in subject_folders:
      subject_files = os.listdir(subject_folder)
      for subject_file in subject_files:
         re_result = filter_re_prog.match(subject_file)
         if re_result is not None:
            bag_file_paths.append(os.path.join(subject_folder,subject_file))

   print 'Going to process these bag files: '
   print bag_file_paths

   for bag_file_path in bag_file_paths:
      print 'Processing bag file: '+bag_file_path
      for topic in topics:
         # Put each topic in a subfolder
         out_sub_dir = topic

         if topic.find('annotation') != -1:
            out_sub_dir = 'annotation'

         if out_sub_dir.startswith('/'):
            out_sub_dir = out_sub_dir[1:]
         out_sub_dir = out_sub_dir.replace('/','_')
         out_dir = os.path.join(out_path, out_sub_dir)
         if not os.path.isdir(out_dir):
            os.makedirs(out_dir)

         # Extract!
         print 'rosrun bag_tools bag_topic_extractor.py '+bag_file_path+' '+topic+' '+out_dir
         os.system('rosrun bag_tools bag_topic_extractor.py '+bag_file_path+' '+topic+' '+out_dir)

   return

if __name__=='__main__':
   if len(sys.argv) > 2:
      bag_path = sys.argv[1]
      out_path = sys.argv[2]
      if len(sys.argv) > 3:
         topics = sys.argv[3].split(',')
      else:
         topics = None
      doExtractStreams(bag_path, out_path, topics)
   else:
      print 'Please provide the following command line arguments:\n1) Path to folder containing bag files\n2) Output directory for extracted stream data\n3) (OPTIONAL) Comma-separated list of topics to extract'
