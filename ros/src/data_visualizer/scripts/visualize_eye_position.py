#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from player_eye_tracking.msg import Position2D

gameVideoImageData = None
eyePosImagePub = None
eyePosImage = Image()
eyePosImageWidth = 480 # Only used if no game video images are being published
eyePosImageHeight = 270 # Only used if no game video images are being published
temp = 1

def gameVideoCallback(gameVideoData):
   global eyePosImagePub
   global gameVideoImageData
   global temp

   eyePosImage.width = gameVideoData.width 
   eyePosImage.height = gameVideoData.height
   eyePosImage.step = gameVideoData.step
   eyePosImage.encoding = gameVideoData.encoding
   eyePosImage.header.frame_id = gameVideoData.header.frame_id
   gameVideoImageData = list(gameVideoData.data);

def eyePositionCallback(eyePositionData):
   global eyePosImagePub
   global gameVideoImageData

   eyePosImage.header.stamp = rospy.get_rostime()
   if (gameVideoImageData is not None):
      eyePosImage.data = gameVideoImageData

   # Put a dot in the image where the eye position lies
   centerX = int(eyePositionData.posX*eyePosImage.width)
   centerY = int(eyePositionData.posY*eyePosImage.height)

   # Add a spot at the eye position
   radius = 5
   valuesPerPixel = eyePosImage.step/eyePosImage.width
   for x in range(centerX-radius, centerX+radius+1):
      for y in range(centerY-radius, centerY+radius+1):
         if x > 0 and x < eyePosImage.width and y > 0 and y < eyePosImage.height:
            pixelIndex = x + y*eyePosImage.width
            replacementList = valuesPerPixel*[0]
            if (eyePosImage.encoding != 'bgr8'):
               replacementList = valuesPerPixel*['\x00']
            eyePosImage.data[valuesPerPixel*pixelIndex:valuesPerPixel*pixelIndex + valuesPerPixel] = replacementList

   if (gameVideoImageData is not None):
      eyePosImage.data = "".join(eyePosImage.data)

   eyePosImagePub.publish(eyePosImage)

   if (gameVideoImageData is None):
      # Return the image to solid color fill
      for x in range(centerX-radius, centerX+radius+1):
         for y in range(centerY-radius, centerY+radius+1):
            if x > 0 and x < eyePosImage.width and y > 0 and y < eyePosImage.height:
               pixelIndex = x + y*eyePosImage.width
               eyePosImage.data[valuesPerPixel*pixelIndex:valuesPerPixel*pixelIndex + valuesPerPixel] = valuesPerPixel*[128]
   

def generateEyePosImage():
   global eyePosImagePub
   eyePosImage.width = eyePosImageWidth
   eyePosImage.height = eyePosImageHeight
   eyePosImage.step = 3*eyePosImageWidth
   eyePosImage.encoding = 'bgr8'
   eyePosImage.header.frame_id = 'invalid'
   eyePosImage.data = [128 for _ in range(eyePosImage.height*eyePosImage.step)]

   rospy.init_node('visualizerEyePosition', anonymous=False)
   gameVideoSub = rospy.Subscriber('/gameVideo', Image, gameVideoCallback)
   eyePosSub = rospy.Subscriber('/eye_position', Position2D, eyePositionCallback)
   eyePosImagePub = rospy.Publisher('eye_position_image', Image, queue_size=1)
   rospy.spin()

if __name__=='__main__':
   try:
      generateEyePosImage()
   except rospy.ROSInterruptException:
      pass
