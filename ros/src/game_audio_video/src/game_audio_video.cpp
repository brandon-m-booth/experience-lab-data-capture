#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "blackmagic_capturer.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "gameAudioVideo");
   ros::NodeHandle nodeHandle;
   ros::Publisher videoPublisher = nodeHandle.advertise<sensor_msgs::Image>("gameVideo", 5);

   BMDPixelFormat pixelFormat = bmdFormat8BitYUV;
   BMDVideoInputFlags videoFlags = bmdVideoInputFlagDefault;
   int audioSampleDepth = 16;
   int numAudioChannels = 2;
   BlackMagicCapturer::GetInstance()->Initialize(pixelFormat, videoFlags, audioSampleDepth, numAudioChannels);

   while (ros::ok())
   {
      Frame frame;
      while (BlackMagicCapturer::GetInstance()->GetNextFrame(frame))
      {
         ROS_INFO("RECEIVED FRAME");
      }
      ros::spinOnce();
   }
}
