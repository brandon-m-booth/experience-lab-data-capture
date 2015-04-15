#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "audio_common_msgs/AudioData.h"
#include "blackmagic_capturer.h"

int main(int argc, char** argv)
{
   // Initialize ROS node and publisher
   ros::init(argc, argv, "gameAudioVideo");
   ros::NodeHandle nodeHandle;
   ros::Publisher videoPublisher = nodeHandle.advertise<sensor_msgs::Image>("gameVideo", 5);
   ros::Publisher audioPublisher = nodeHandle.advertise<audio_common_msgs::AudioData>("gameAudio", 10);

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
            image.height = frame.videoHeight;
            image.width = frame.videoStride/numChannels; // and one byte per pixel
            image.encoding = encoding;
            image.is_bigendian = 0; // False
            image.step = frame.videoStride;
            ROS_DEBUG("Num Channels: %d, Stride: %d, Height: %d,", numChannels, (int)frame.videoStride, (int)frame.videoHeight);
            image.data.insert(image.data.begin(), &frame.videoBytes[0], &frame.videoBytes[frame.videoStride*frame.videoHeight]);

            videoPublisher.publish(image);
         }

         if (frame.audioBytes)
         {
            audio_common_msgs::AudioData audioData;
            audioData.data.insert(audioData.data.begin(), &frame.audioBytes[0], &frame.audioBytes[frame.numAudioBytes]);

            audioPublisher.publish(audioData);
         }

         ++frameCounter;
      }
      ros::spinOnce();
   }
}
