#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "player_eye_tracking/Position2D.h"

const unsigned int EyePosImageWidth = 480;
const unsigned int EyePosImageHeight = 270;
std::vector<uint8_t> gameVideoImageData;
sensor_msgs::Image eyePosImage;

void gameVideoCallback(const sensor_msgs::Image::ConstPtr& gameVideoImage)
{
   eyePosImage.width = gameVideoImage->width;
   eyePosImage.height = gameVideoImage->height;
   eyePosImage.step = gameVideoImage->step;
   eyePosImage.encoding = gameVideoImage->encoding;
   eyePosImage.header.frame_id = gameVideoImage->header.frame_id;

   // Make sure our game video image buffer has enough space to hold
   // the data and then copy it
   uint32_t imageBufferSize = gameVideoImage->height*gameVideoImage->step;
   if (imageBufferSize > gameVideoImageData.capacity())
   {
      gameVideoImageData.reserve(imageBufferSize);
   }
   gameVideoImageData.clear();
   gameVideoImageData.insert(gameVideoImageData.begin(), &gameVideoImage->data[0], &gameVideoImage->data[imageBufferSize+1]);
}

void eyePositionCallback(const player_eye_tracking::Position2D::ConstPtr& eyePosition2D)
{
   eyePosImage.header.stamp = ros::Time::now();
   if (gameVideoImageData.size() > 0)
   {
      eyePosImage.data.clear();
      eyePosImage.data.insert(eyePosImage.data.begin(), &gameVideoImageData[0], &gameVideoImageData.back());
   }

   // Put a dot in the image at the eye position
   const int dotSizePixels = 5;
   int centerX = eyePosition2D->posX*eyePosImage.width;
   int centerY = eyePosition2D->posY*eyePosImage.height;
   unsigned int valuesPerPixel = eyePosImage.step/eyePosImage.width;
   for (int x = centerX-dotSizePixels; x <= centerX+dotSizePixels; ++x)
   {
      for (int y = centerY-dotSizePixels; y <= centerY+dotSizePixels; ++y)
      {
         if (x > 0 && x < eyePosImage.width && y > 0 && y < eyePosImage.height)
         {
            unsigned int pixelIndex = (unsigned int)(x + y*eyePosImage.width);
            for (unsigned int i = valuesPerPixel*pixelIndex; i < valuesPerPixel*pixelIndex + valuesPerPixel; ++i)
            {
               eyePosImage.data[i] = 0;
            }
         }
      }
   }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizerEyePosition");
	ros::NodeHandle nodeHandle;
	ros::Subscriber gameVideoSub = nodeHandle.subscribe("/gameVideo", 1, gameVideoCallback);
   ros::Subscriber eyePosSub = nodeHandle.subscribe("/eye_position", 1, eyePositionCallback);
   ros::Publisher eyePosImagePub = nodeHandle.advertise<sensor_msgs::Image>("eye_position_image", 1);

   eyePosImage.width = EyePosImageWidth;
   eyePosImage.height = EyePosImageHeight;
   eyePosImage.step = 3*eyePosImage.width;
   eyePosImage.encoding = "bgr8";
   eyePosImage.header.frame_id = "invalid";
   eyePosImage.data.reserve(eyePosImage.height*eyePosImage.step);
   memset(&eyePosImage.data[0], 128, eyePosImage.step*eyePosImage.height); // Default image background

   while (ros::ok())
   {
      eyePosImagePub.publish(eyePosImage);
      if (gameVideoImageData.size() == 0)
      {
         // If no game video image has been received, then paint a solid color background
         memset(&eyePosImage.data[0], 128, eyePosImage.step*eyePosImage.height);
      }

      ros::spinOnce();
   }
}
