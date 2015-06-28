#include "ros/ros.h"
#include "std_msgs/String.h"
#include "player_input/CursorPosition.h"
#include "inputEvent.h"
#include <zmq.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "playerInput");
   ros::NodeHandle nodeHandle;
   ros::Publisher inputKeyEventPublisher = nodeHandle.advertise<std_msgs::String>("playerInput/keyEvents", 50);
   ros::Publisher inputCursorPosPublisher = nodeHandle.advertise<player_input::CursorPosition>("playerInput/cursorPosition", 50);

   // Setup ZMQ and subscriber socket
   int port = 37481;
   //nodeHandle.getParam("port", port);
   void* zmqContext = zmq_ctx_new();
   void* zmqSocket = zmq_socket(zmqContext, ZMQ_SUB);
   std::string bindString = "tcp://*:";
   bindString += std::to_string(port);
   zmq_bind(zmqSocket, bindString.c_str());
   zmq_setsockopt(zmqSocket, ZMQ_SUBSCRIBE, NULL, 0);
   
   while (ros::ok())
   {
      Input::InputEvent inputEvent;
      int bytesReceived = zmq_recv(zmqSocket, &inputEvent, sizeof(inputEvent), ZMQ_DONTWAIT);
      if (bytesReceived < 0)
      {
         if (errno != EAGAIN && errno != EINTR)
         {
            ROS_ERROR("ZMQ receive message failed with errno: %d", errno);
         }
      }
      else
      {
         if (bytesReceived != sizeof(inputEvent))
         {
            std::string errorMsg = "Message received, but expected " + std::to_string(sizeof(inputEvent));
            errorMsg += " bytes and got " + std::to_string(bytesReceived) + " bytes instead. FIX ME";
            ROS_ERROR("%s", errorMsg.c_str());
         }
         else
         {
            std::string key;
            std::string value;
            if (inputEvent.inputEventType != Input::CursorPosition)
            {
               Input::GetKeyValueStrings(key, value, inputEvent);
               std_msgs::String inputKeyEventMessage;
               inputKeyEventMessage.data = key + " " + value;
               inputKeyEventPublisher.publish(inputKeyEventMessage);
            }
            else
            {
               player_input::CursorPosition inputCursorPosMessage;
               inputCursorPosMessage.posX = inputEvent.inputDataItem1;
               inputCursorPosMessage.posY = inputEvent.inputDataItem2;
               inputCursorPosPublisher.publish(inputCursorPosMessage);
            }
         }
      }
      
	   ros::spinOnce();
   }

   zmq_ctx_destroy(zmqContext);
}
