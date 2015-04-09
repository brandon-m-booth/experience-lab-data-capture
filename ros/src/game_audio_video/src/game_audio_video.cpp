#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "blackmagic_capturer.h"

int main(int argc, char** argv)
{
   // Initialize ROS node and publisher
   ros::init(argc, argv, "gameAudioVideo");
   ros::NodeHandle nodeHandle;
   ros::Publisher videoPublisher = nodeHandle.advertise<sensor_msgs::Image>("gameVideo", 5);

   // Initialize Black Magic capturer
   BMDPixelFormat pixelFormat = bmdFormat8BitYUV;
   BMDVideoInputFlags videoFlags = bmdVideoInputFlagDefault;
   int audioSampleDepth = 16;
   int numAudioChannels = 2;
   if (!BlackMagicCapturer::GetInstance()->Initialize(pixelFormat, videoFlags, audioSampleDepth, numAudioChannels))
   {
      ROS_ERROR("%s", BlackMagicCapturer::GetInstance()->GetError().c_str());
   }

   // Loop and capture frames
   while (ros::ok())
   {
      BlackMagicCapturer::GetInstance()->Update();

      Frame frame;
      while (BlackMagicCapturer::GetInstance()->GetNextFrame(frame))
      {
         ROS_INFO("RECEIVED FRAME");
      }
      ros::spinOnce();
   }
}
