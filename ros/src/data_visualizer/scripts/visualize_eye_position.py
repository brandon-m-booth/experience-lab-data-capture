#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from player_eye_tracking.msg import Position2D

eyePosImage = Image()
eyePosImagePub = None
eyePosImageWidth = 480
eyePosImageHeight = 270

def eyePositionCallback(eyePositionData):
   global eyePosImagePub

   eyePosImage.header.stamp = rospy.get_rostime()

   # Put a dot in the image where the eye position lies
   centerX = int(eyePositionData.posX*eyePosImage.width)
   centerY = int(eyePositionData.posY*eyePosImage.height)

   # Add a black spot at the eye position
   radius = 5
   for x in range(centerX-radius, centerX+radius+1):
      for y in range(centerY-radius, centerY+radius+1):
         if x > 0 and x < eyePosImage.width and y > 0 and y < eyePosImage.height:
            pixelIndex = x + y*eyePosImage.width
            eyePosImage.data[3*pixelIndex:3*pixelIndex + 3] = 3*[0]

   eyePosImagePub.publish(eyePosImage)

   # Return the image to solid color fill
   for x in range(centerX-radius, centerX+radius+1):
      for y in range(centerY-radius, centerY+radius+1):
         if x > 0 and x < eyePosImage.width and y > 0 and y < eyePosImage.height:
            pixelIndex = x + y*eyePosImage.width
            eyePosImage.data[3*pixelIndex:3*pixelIndex + 3] = 3*[128]
   

def generateEyePosImage():
   global eyePosImagePub
   eyePosImage.width = eyePosImageWidth
   eyePosImage.height = eyePosImageHeight
   eyePosImage.step = 3*eyePosImageWidth
   eyePosImage.encoding = 'bgr8'
   eyePosImage.header.frame_id = 'invalid'
   eyePosImage.data = [128 for _ in range(eyePosImage.height*eyePosImage.step)]

   rospy.init_node('visualizerEyePosition', anonymous=False)
   eyePosSub = rospy.Subscriber('/eye_position', Position2D, eyePositionCallback)
   eyePosImagePub = rospy.Publisher('eye_position_image', Image, queue_size=1)
   rospy.spin()

if __name__=='__main__':
   try:
      generateEyePosImage()
   except rospy.ROSInterruptException:
      pass
