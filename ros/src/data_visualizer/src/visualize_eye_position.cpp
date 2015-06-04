#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "player_eye_tracking/Position2D.h"

const unsigned int EyePosImageWidth = 1920;
const unsigned int EyePosImageHeight = 1080;
std::vector<uint8_t> gameVideoImageData;
sensor_msgs::Image eyePosImage;
int eyePosX;
int eyePosY;
bool hasReceivedNewVideoFrame = false;
ros::Time lastReceivedEyePosTime;

void GameVideoCallback(const sensor_msgs::Image::ConstPtr& gameVideoImage)
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

   eyePosImage.header.stamp = ros::Time::now();
   hasReceivedNewVideoFrame = true;
}

void EyePositionCallback(const player_eye_tracking::Position2D::ConstPtr& eyePosition2D)
{
   eyePosX = eyePosition2D->posX*eyePosImage.width;
   eyePosY = eyePosition2D->posY*eyePosImage.height;
   lastReceivedEyePosTime = ros::Time::now();
}

void GetDotColor(std::vector<int>& pixelValues, const std::string& encoding, const ros::Time& lastReceivedEyePosTime)
{
   const ros::Duration DotValidDuration(0.1);
   bool isDotValid = (ros::Time::now() - lastReceivedEyePosTime) < DotValidDuration;
   if (encoding == "yuv422")
   {
      if (isDotValid)
      {
         pixelValues.push_back(43);
         pixelValues.push_back(169);
         pixelValues.push_back(28);
         pixelValues.push_back(169);
      }
      else
      {
         pixelValues.push_back(110);
         pixelValues.push_back(76);
         pixelValues.push_back(218);
         pixelValues.push_back(76);
      }
   }
   else if (encoding == "bgr8")
   {
      if (isDotValid)
      {
         pixelValues.push_back(0);
         pixelValues.push_back(255);
         pixelValues.push_back(0);
      }
      else
      {
         pixelValues.push_back(0);
         pixelValues.push_back(0);
         pixelValues.push_back(255);
      }
   }
   else
   {
      ROS_ERROR("Unknown encoding type: %s. Colors will be incorrect, please FIX ME!", encoding.c_str());
   }
}

int GetBackgroundColor(const std::string& encoding)
{
   if (encoding == "yuv422")
   {
      return 0x80;
   }
   else if (encoding == "bgr8")
   {
      return 128;
   }
   else
   {
      ROS_WARN("Unknown encoding type: %s. Colors will be incorrect, please FIX ME!", encoding.c_str());
      return 128;
   }
}

void AddEyePositionDotToImage()
{
   if (gameVideoImageData.size() > 0)
   {
      // Copy the game video image
      eyePosImage.data.clear();
      eyePosImage.data.insert(eyePosImage.data.begin(), &gameVideoImageData[0], &gameVideoImageData.back());
   }
   else
   {
      // If no game video image has been received, then paint a solid color background
      int backgroundColor = GetBackgroundColor(eyePosImage.encoding);
      eyePosImage.data.resize(eyePosImage.step*eyePosImage.height);
      memset(&eyePosImage.data[0], backgroundColor, eyePosImage.step*eyePosImage.height);
   }

   // Put a dot in the image at the eye position
   const int dotSizePixels = 5;
   std::vector<int> dotPixelValues;
   unsigned int valuesPerPixel = eyePosImage.step/eyePosImage.width;
   GetDotColor(dotPixelValues, eyePosImage.encoding, lastReceivedEyePosTime);
   for (int x = eyePosX-dotSizePixels; x <= eyePosX+dotSizePixels; ++x)
   {
      for (int y = eyePosY-dotSizePixels; y <= eyePosY+dotSizePixels; ++y)
      {
         if (x > 0 && x < eyePosImage.width && y > 0 && y < eyePosImage.height)
         {
            unsigned int pixelIndex = (unsigned int)(x + y*eyePosImage.width);
            unsigned int encodedPixelIndex = valuesPerPixel*pixelIndex;
            for (unsigned int i = 0; i < valuesPerPixel; ++i)
            {
               if (eyePosImage.encoding == "yuv422") // yuv422 encodes pairs of pixels
               {
                  if (pixelIndex%2 == 0)
                  {
                     eyePosImage.data[encodedPixelIndex+i] = dotPixelValues[i];
                  }
                  else
                  {
                     eyePosImage.data[encodedPixelIndex+i] = dotPixelValues[i+2];
                  }
               }
               else
               {
                  eyePosImage.data[encodedPixelIndex+i] = dotPixelValues[i];
               }
            }
         }
      }
   }
}

int main(int argc, char** argv)
{
   ros::Duration BailWaitingForGameVideoDuration(1, 0);

	ros::init(argc, argv, "visualizerEyePosition");
	ros::NodeHandle nodeHandle;
	ros::Subscriber gameVideoSub = nodeHandle.subscribe("/gameVideo", 1, GameVideoCallback);
   ros::Subscriber eyePosSub = nodeHandle.subscribe("/eye_position", 1, EyePositionCallback);
   ros::Publisher eyePosImagePub = nodeHandle.advertise<sensor_msgs::Image>("eye_position_image", 1);
   ros::Duration updateDurationWithoutGameVideo(0.0333333); // 30 Hertz

   eyePosImage.width = EyePosImageWidth;
   eyePosImage.height = EyePosImageHeight;
   eyePosImage.step = 3*eyePosImage.width;
   eyePosImage.encoding = "bgr8";
   eyePosImage.header.frame_id = "invalid";
   eyePosImage.data.resize(eyePosImage.height*eyePosImage.step);

   while (ros::ok())
   {
      AddEyePositionDotToImage();
      eyePosImagePub.publish(eyePosImage);
      
      if (gameVideoImageData.size() == 0)
      {
         ros::Time startTime = ros::Time::now();
         while (ros::Time::now() - startTime < updateDurationWithoutGameVideo)
         {
            ros::spinOnce();
         }
      }
      else
      {
         ros::Time startTime = ros::Time::now();
         while (!hasReceivedNewVideoFrame)
         {
            ros::spinOnce();

            // If we don't see a game video frame for a while, bail
            // and revert to the default output
            if (ros::Time::now() - startTime > BailWaitingForGameVideoDuration)
            {
               gameVideoImageData.clear();
               break;
            }
         }
         hasReceivedNewVideoFrame = false;
      }
   }
}
