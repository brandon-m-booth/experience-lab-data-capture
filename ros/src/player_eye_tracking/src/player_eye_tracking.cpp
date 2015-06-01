#include "ros/ros.h"
#include "player_eye_tracking/Position2D.h"
#include "tobii_eye_tracker.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "playerEyeTracking");
	ros::NodeHandle nodeHandle;
	ros::Publisher eyeTrackingPublisher = nodeHandle.advertise<player_eye_tracking::Position2D>("eye_position", 1);

   TobiiEyeTracker::GetInstance()->Initialize();
   const std::string& errorString = TobiiEyeTracker::GetInstance()->GetErrorString();
   if (!errorString.empty())
   {
      ROS_ERROR("%s", errorString.c_str());
      return EXIT_FAILURE;
   }
   ROS_INFO("Eye tracking started...");
   ROS_INFO("TobiiGazeCore version: %s", TobiiEyeTracker::GetInstance()->GetSDKVersionString().c_str());
   ROS_INFO("Eye tracking device serial number is: %s", TobiiEyeTracker::GetInstance()->GetDeviceSerialNumber().c_str());

   while (ros::ok())
   {
      while (TobiiEyeTracker::GetInstance()->HasGazeData())
      {
         GazeData gazeData;
         TobiiEyeTracker::GetInstance()->GetGazeData(gazeData);

         // Compute the average gaze location between both the left and right eyes
         player_eye_tracking::Position2D eyePosition;
         if (gazeData.leftEyeX != 0 && gazeData.leftEyeY != 0)
         {
            if (gazeData.rightEyeX != 0 && gazeData.rightEyeY != 0)
            {
               eyePosition.posX = (gazeData.leftEyeX + gazeData.rightEyeX)/2.0f;
               eyePosition.posY = (gazeData.leftEyeY + gazeData.rightEyeY)/2.0f;
            }
            else
            {
               eyePosition.posX = gazeData.leftEyeX;
               eyePosition.posY = gazeData.leftEyeY;
            }

            eyeTrackingPublisher.publish(eyePosition);
         }
         else if (gazeData.rightEyeX != 0 && gazeData.rightEyeY != 0)
         {
            eyePosition.posX = gazeData.rightEyeX;
            eyePosition.posY = gazeData.rightEyeY;

            eyeTrackingPublisher.publish(eyePosition);
         }

      }

      ros::spinOnce();
   }

   TobiiEyeTracker::GetInstance()->Shutdown();
   ROS_INFO("Eye tracking stopped...");
}
