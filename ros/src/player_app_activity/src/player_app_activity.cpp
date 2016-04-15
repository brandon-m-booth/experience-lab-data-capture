#include "ros/ros.h"
#include "std_msgs/String.h"
#include "appActivityEvent.h"
#include <zmq.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "playerAppActivity");
   ros::NodeHandle nodeHandle;
   ros::Publisher appActivityEventPublisher = nodeHandle.advertise<std_msgs::String>("player_app_activity", 20);

   // Setup ZMQ and subscriber socket
   int port = 37482;
   //nodeHandle.getParam("port", port);
   void* zmqContext = zmq_ctx_new();
   void* zmqSocket = zmq_socket(zmqContext, ZMQ_SUB);
   std::string bindString = "tcp://*:";
   bindString += std::to_string(port);
   zmq_bind(zmqSocket, bindString.c_str());
   zmq_setsockopt(zmqSocket, ZMQ_SUBSCRIBE, NULL, 0);
   
   while (ros::ok())
   {
      AppActivity::AppActivityEvent appActivityEvent;
      int bytesReceived = zmq_recv(zmqSocket, &appActivityEvent, sizeof(appActivityEvent), ZMQ_DONTWAIT);
      if (bytesReceived < 0)
      {
         if (errno != EAGAIN && errno != EINTR)
         {
            ROS_ERROR("ZMQ receive message failed with errno: %d", errno);
         }
      }
      else
      {
         if (bytesReceived != sizeof(appActivityEvent))
         {
            std::string errorMsg = "Message received, but expected " + std::to_string(sizeof(appActivityEvent));
            errorMsg += " bytes and got " + std::to_string(bytesReceived) + " bytes instead. FIX ME";
            ROS_ERROR("%s", errorMsg.c_str());
         }
         else
         {
            std::string key;
            std::string value;
            if (appActivityEvent.appActivityEventType == AppActivity::ActiveApplication)
            {
               std_msgs::String appActivityEventMessage;
               appActivityEventMessage.data = appActivityEvent.dataItem;
               appActivityEventPublisher.publish(appActivityEventMessage);
            }
         }
      }
      
	   ros::spinOnce();
   }

   zmq_ctx_destroy(zmqContext);
}
