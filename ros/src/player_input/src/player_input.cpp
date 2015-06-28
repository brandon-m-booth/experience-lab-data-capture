#include "ros/ros.h"
#include "std_msgs/String.h"
#include "inputEvent.h"
#include <zmq.h>
#include <string>

void GetKeyValueStrings(std::string& key, std::string& value, const Input::InputEvent& inputEvent);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "playerInput");
   ros::NodeHandle nodeHandle;
   ros::Publisher inputEventPublisher = nodeHandle.advertise<std_msgs::String>("playerInput", 50);

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
         if (errno != EAGAIN)
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
            GetKeyValueStrings(key, value, inputEvent);
            std_msgs::String inputEventMessage;
            inputEventMessage.data = key + " " + value;
            inputEventPublisher.publish(inputEventMessage);//inputEventString.c_str());
         }
      }
      
	   ros::spinOnce();
   }

   zmq_ctx_destroy(zmqContext);
}

void GetKeyValueStrings(std::string& key, std::string& value, const Input::InputEvent& inputEvent)
{
   const char* const keys[] =
   {
      "N/A",
      "MouseLeft",
      "MouseRight",
      "Cancel",
      "MouseWheel",
      "MouseX1",
      "MouseX2",
      "N/A",
      "Backspace",
      "Tab",
      "N/A",
      "N/A",
      "Clear",
      "Enter",
      "N/A",
      "N/A",
      "Shift",
      "Control",
      "Alt",
      "Pause",
      "CapsLock",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "Escape",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "Space",
      "PageUp",
      "PageDown",
      "End",
      "Home",
      "Left",
      "Up",
      "Right",
      "Down",
      "Select",
      "Print",
      "Execute",
      "PrintScreen",
      "Insert",
      "Delete",
      "Help",
      "Zero",
      "One",
      "Two",
      "Three",
      "Four",
      "Five",
      "Six",
      "Seven",
      "Eight",
      "Nine",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "A",
      "B",
      "C",
      "D",
      "E",
      "F",
      "G",
      "H",
      "I",
      "J",
      "K",
      "L",
      "M",
      "N",
      "O",
      "P",
      "Q",
      "R",
      "S",
      "T",
      "U",
      "V",
      "W",
      "X",
      "Y",
      "Z",
      "LeftSpecialKey",
      "RightSpecialKey",
      "ApplicationsKey",
      "N/A",
      "Sleep",
      "NumPad0",
      "NumPad1",
      "NumPad2",
      "NumPad3",
      "NumPad4",
      "NumPad5",
      "NumPad6",
      "NumPad7",
      "NumPad8",
      "NumPad9",
      "Multiply",
      "Add",
      "Seperator",
      "Subtract",
      "Decimal",
      "Divide",
      "F1",
      "F2",
      "F3",
      "F4",
      "F5",
      "F6",
      "F7",
      "F8",
      "F9",
      "F10",
      "F11",
      "F12",
      "F13",
      "F14",
      "F15",
      "F16",
      "F17",
      "F18",
      "F19",
      "F20",
      "F21",
      "F22",
      "F23",
      "F24",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "Numlock",
      "ScrollLock",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "LeftShift",
      "RightShift",
      "LeftControl",
      "RightContol",
      "LeftAlt",
      "RightAlt",
      "BrowserBack",
      "BrowserForward",
      "BrowserRefresh",
      "BrowserStop",
      "BrowserSearch",
      "BrowserFavorites",
      "BrowserHome",
      "VolumeMute",
      "VolumeDown",
      "VolumeUp",
      "NextTrack",
      "PreviousTrack",
      "StopMedia",
      "PlayPause",
      "LaunchMail",
      "SelectMedia",
      "LaunchApp1",
      "LaunchApp2",
      "N/A",
      "N/A",
      "Semicolon",
      "Equals",
      "Comma",
      "Minus",
      "Period",
      "ForwardSlash",
      "Backtick",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "LeftBracket",
      "BackSlash",
      "RightBracket",
      "Apostrophe",
      "OEM8",
      "N/A",
      "N/A",
      "OEM102",
      "N/A",
      "N/A",
      "Process",
      "N/A",
      "Packet",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "N/A",
      "Attn",
      "CrSel",
      "ExSel",
      "EraseEOF",
      "Play",
      "Zoom",
      "N/A",
      "PA1",
      "OEMClear",
   };

   if (inputEvent.inputEventType == Input::KeyState)
   {
      key = keys[inputEvent.inputDataItem1];
      if (inputEvent.inputDataItem2 == Input::DownState)
      {
         value = "Down";
      }
      else if (inputEvent.inputDataItem2 == Input::UpState)
      {
         value = "Up";
      }
      else
      {
         value = std::to_string(inputEvent.inputDataItem2);
      }
   }
   else if (inputEvent.inputEventType == Input::CursorPosition)
   {
      key = "Mouse Move";
      value = "(" + std::to_string(inputEvent.inputDataItem1) + ", " + std::to_string(inputEvent.inputDataItem2) + ")";
   }
   else if (inputEvent.inputEventType == Input::MouseWheelScroll)
   {
      key = "Mouse Wheel Scroll";
      value = std::to_string(inputEvent.inputDataItem2);
   }
   else
   {
      key = value = "ERROR";
      ROS_ERROR("Error: Unknown input event type. FIX ME");
   }
}
