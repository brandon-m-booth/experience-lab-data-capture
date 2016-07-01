#!/usr/bin/python

import os
import re
import pdb
import platform

def doExtractStreams():
   topics = ['/audio', '/eye_position', '/kinect2/rgb_rect/image', '/game_video/rgb/image', '/playerInput/cursorPosition', '/playerInput/keyEvents', '/player_app_activity', '/player_eeg', '/player_video/overhead/camera/image_mono']
   for annotator_id in range(15):
      for session in ['lecture', 'student']:
         for stream in ['engagement', 'isTakingNotes']:
            topics.append('annotation/'+session+'/annotator'+str(annotator_id)+'/'+stream)
   filter_re_prog = re.compile('.*[main|main2|b1|b2]')
   bag_file_paths = []

   # Get list of all subject file paths in all subject subfolders
   base_path = '/media/brandon/SailData/MOOC_Engagement_Pilot_Study/stitched_bags/'
   out_dir_base = '/USC/2016_Engagement_Pilot/data/'
   for subject_file in os.listdir(base_path):
      bag_file_paths.append(base_path+'/'+subject_file)
   #for subject_folder in subject_folders:
   #   subject_files = os.listdir(base_path+subject_folder)
   #   for subject_file in subject_files:
   #      re_result = filter_re_prog.match(subject_file)
   #      if re_result is not None:
   #         bag_file_paths.append(base_path+subject_folder+'/'+subject_file)

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
         if not os.path.isdir(out_dir_base+out_sub_dir):
            os.mkdir(out_dir_base+out_sub_dir)

         # Extract!
         print 'rosrun bag_tools bag_topic_extractor.py '+bag_file_path+' '+topic+' '+out_dir_base+out_sub_dir
         os.system('rosrun bag_tools bag_topic_extractor.py '+bag_file_path+' '+topic+' '+out_dir_base+out_sub_dir)

   return

if __name__=='__main__':
   doExtractStreams()
