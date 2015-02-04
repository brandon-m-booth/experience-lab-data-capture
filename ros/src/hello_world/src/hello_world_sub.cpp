#include "ros/ros.h"
#include "std_msgs/String.h"

void topicCallback(const std_msgs::String::ConstPtr& message)
{
	ROS_INFO("Someone said: %s", message->data.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "helloListener");
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub = nodeHandle.subscribe("hello", 10, topicCallback);
	ros::spin();
}
