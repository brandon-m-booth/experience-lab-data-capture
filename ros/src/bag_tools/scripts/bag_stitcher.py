#!/usr/bin/env python

import platform
import os
import pdb
import rospy
import rosbag
from std_msgs.msg import Float32

def stitchAnnotatorBags(bag, annotator_bags, annotator_ids, out_bag):
   # Obtain a sorted list of new topics to stitch in
   stitchList = []
   for i in range(len(annotator_bags)):
      annotator_bag = annotator_bags[i]
      annotator_id = annotator_ids[i]
      bagStartTime = rospy.Time(bag.get_start_time())
      lastTime = bagStartTime
      for topic, msg, t in annotator_bag.read_messages():
         if t < lastTime and len(stitchList) > 0:
            print 'DOES THIS EVER HAPPEN?!?!'
            clipIndex = next(x[0] for x in enumerate(stitchList) if x[1][2] >= t)
            stitchList = stitchList[0:clipIndex]
         lastTime = t
         stitchList.append((topic, msg, t, annotator_id))

   # Write original topics
   for topic, msg, t in bag.read_messages():
      out_bag.write(topic, msg, t)
      
   # Stitch in new topics
   for (topic, msg, t, annotator_id) in stitchList:
      if topic.startswith('annotation'):
         topic_suffix = '/'.join(topic.split('/')[1:])
         topic = 'annotation/annotator'+str(annotator_id)+'/'+topic_suffix
      out_bag.write(topic, msg, t)


def doAnnotationStitching():
   rospy.init_node('bag_stitcher')

   try:
      input_bags_path = rospy.get_param('input_bags_path')
      input_annotated_bag_path = rospy.get_param('input_annotated_bags_path')
      output_stitched_bag_path = rospy.get_param('output_stitched_bags_path')
   except KeyError:
      rospy.logerr('Either ROS param "input_bags_path", "input_annotated_bags_path", or "output_stitched_bags_path" was not set. Shutting down!')
      return

   input_bags_subfolders = os.listdir(input_bags_path)
   found_subject_subfolder = False
   for input_bags_subfolder in input_bags_subfolders:
      if input_bags_subfolder.startswith('subject_'):
         found_subject_subfolder = True
         break
   if not found_subject_subfolder:
      print 'Please provide a valid data folder containing subfolders with names like "subject_01"'
      return

   if not os.path.isdir(output_stitched_bag_path):
      os.mkdir(output_stitched_bag_path)

   annotated_bag_filenames = os.listdir(input_annotated_bag_path)

   # Count the number of unique user names
   user_names = []
   subject_id_tasks = []
   for annotated_bag_filename in annotated_bag_filenames:
      split_bag_filename = annotated_bag_filename.split('_')
      subject_id = int(split_bag_filename[0])
      task = split_bag_filename[1]
      user_name = split_bag_filename[-1][:-4]
      subject_id_tasks.append((subject_id, task))
      user_names.append(user_name)
   unique_subject_id_tasks = list(set(subject_id_tasks))
   unique_user_names = list(set(user_names))

   for unique_subject_id_task in unique_subject_id_tasks:
      # Find the original bag corresponding to this subject id and task
      stitch_bag_source = None
      input_bags = os.listdir(input_bags_path+'/subject_%02d'%unique_subject_id_task[0])
      for input_bag in input_bags:
         input_bag_task = input_bag.split('_')[-1][:-4]
         if input_bag_task == unique_subject_id_task[1]:
            stitch_bag_filename = input_bag
            stitch_bag_source = input_bags_path+'/subject_%02d/'%(unique_subject_id_task[0])+'/'+input_bag
            break

      if stitch_bag_source is None:
         rospy.logerr('Unable to find suitable source stitch file. FIX ME!')
         continue

      
      # Stitch in the correponding annotator bags
      outbag_path = output_stitched_bag_path+'/'+stitch_bag_filename[:-4]+'_annotated.bag'
      print 'Processing '+outbag_path+'...'
      with rosbag.Bag(outbag_path, 'w') as out_bag:
         stitch_bags = []
         unique_annotator_ids = []
         for annotated_bag_filename in annotated_bag_filenames:
            split_filename = annotated_bag_filename.split('_')
            subject_id = int(split_filename[0])
            task = split_filename[1]
            if unique_subject_id_task[0] == subject_id and unique_subject_id_task[1] == task:
               user_name = split_filename[-1][:-4]
               unique_annotator_id = unique_user_names.index(user_name)
               for input_bag in input_bags:
                  input_bag_task = input_bag.split('_')[-1][:-4]
                  if input_bag_task == task:
                     stitch_bag = rosbag.Bag(input_annotated_bag_path+'/'+annotated_bag_filename, 'r', allow_unindexed=True)
                     stitch_bags.append(stitch_bag) # Push this bag and annotator id into the list of things to process
                     unique_annotator_ids.append(unique_annotator_id)

         # Stitch 'em!
         source_bag = rosbag.Bag(stitch_bag_source, 'r', allow_unindexed=True)
         stitchAnnotatorBags(source_bag, stitch_bags, unique_annotator_ids, out_bag)
                     
if __name__=='__main__':
   doAnnotationStitching()
