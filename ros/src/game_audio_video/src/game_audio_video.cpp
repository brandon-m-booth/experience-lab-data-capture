#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
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
   uint32_t frameCounter = 0;
   while (ros::ok())
   {
      BlackMagicCapturer::GetInstance()->Update();

      Frame frame;
      while (BlackMagicCapturer::GetInstance()->GetNextFrame(frame))
      {
         ROS_DEBUG("RECEIVED FRAME");
   
         if (frame.videoBytes)
         {
            std::string encoding = sensor_msgs::image_encodings::YUV422;
            int numChannels = sensor_msgs::image_encodings::numChannels(encoding);
            sensor_msgs::Image image;
            image.header.seq = frameCounter;
            image.header.stamp = ros::Time::now();
            image.header.frame_id = "0"; // No frame
            image.height = frame.height;
            image.width = frame.stride/numChannels; // and one byte per pixel
            image.encoding = encoding;
            image.is_bigendian = 0; // False
            image.step = frame.stride;
            ROS_DEBUG("Num Channels: %d, Stride: %d, Height: %d,", numChannels, (int)frame.stride, (int)frame.height);
            image.data.insert(image.data.begin(), &frame.videoBytes[0], &frame.videoBytes[frame.stride*frame.height]);

            videoPublisher.publish(image);
         }

         ++frameCounter;
      }
      ros::spinOnce();
   }
}
