#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import Image
from fake_data_generator.msg import Position2D
from fake_data_generator.msg import EEGData

# This file is intended to help produce fake sensor data until enough code is in place
# that real sensor data can be produced. This fake data generator checks to see if a
# kinect_node is running and then uses it to generate the kinect data rather than 
# generating fake data.  All other sensor data streams are faked currently. To run
# the kinect_node, type:
#
#    roslaunch freenect_launch freenect.launch
#
# Note: this will only work for Kinect v1 devices
rgbImage = Image()
depthImage = Image()

def rgbImageCallback(imageData):
	global rgbImage
	rgbImage = imageData

def depthImageCallback(depthData):
	global depthImage
	depthImage = depthData

def generateFakeVideoImages():
	global rgbImage
	rgbImage.height = 480
	rgbImage.width = 640
	rgbImage.step = 1920
	rgbImage.encoding = 'bgr8'
	rgbImage.header.frame_id = '/camera_rgb_optical_frame'
	rgbImage.header.stamp = rospy.get_rostime()
	rgbImage.data = [random.randint(0, 255) for _ in xrange(rgbImage.height*rgbImage.width*3)]

	global depthImage
	depthImage.height = 480
	depthImage.width = 640
	depthImage.step = 2560
	depthImage.encoding = '32FC1'
	depthImage.header.frame_id = '/camera_depth_optical_frame'
	depthImage.header.stamp = rospy.get_rostime()
	depthImage.data = [random.randint(0, 255) for _ in xrange(depthImage.height*depthImage.width*4)]

def generateFakeSensorData():
	# Attempt to find a running Kinect sensor and use its data stream if present,
	# otherwise fake the camera data
	rospy.init_node('fakeDataGeneratorNode', anonymous=True)
	kinect_rgb_image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, rgbImageCallback)
	kinect_depth_image_sub = rospy.Subscriber('/camera/depth/image_rect', Image, depthImageCallback)
	rospy.sleep(1)
	use_kinect = (kinect_rgb_image_sub.get_num_connections() > 0)

	# Setup fake data publishers
	fake_rgb_video_pub = rospy.Publisher('rgb_video', Image, queue_size=1)
	fake_depth_video_pub = rospy.Publisher('depth_video', Image, queue_size=1)
	fake_eye_tracking_pub = rospy.Publisher('eye_tracking', Position2D, queue_size=1)
	fake_eeg_pub = rospy.Publisher('eeg', EEGData, queue_size=1)
	
	if not use_kinect:
		generateFakeVideoImages()

	rate = rospy.Rate(15) # Repeat every Hertz
	while not rospy.is_shutdown():

		eyePos = Position2D()
		eyePos.posX = random.randint(0, 2**32)
		eyePos.posY = random.randint(0, 2**32)

		eegData = EEGData()
		eegData.e1 = random.random()
		eegData.e2 = random.random()
		eegData.e3 = random.random()
		eegData.e4 = random.random()
		eegData.e5 = random.random()
		eegData.e6 = random.random()
		eegData.e7 = random.random()
		eegData.e8 = random.random()
		eegData.e9 = random.random()
		eegData.e10 = random.random()
		eegData.e11 = random.random()
		eegData.e12 = random.random()
		eegData.e13 = random.random()
		eegData.e14 = random.random()

		fake_rgb_video_pub.publish(rgbImage)
		fake_depth_video_pub.publish(depthImage)
		fake_eye_tracking_pub.publish(eyePos)
		fake_eeg_pub.publish(eegData)

		rate.sleep()

if __name__=='__main__':
	try:
		generateFakeSensorData()
	except rospy.ROSInterruptException:
		pass
