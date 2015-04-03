#include "ros/ros.h"
#include "sensor_msgs/Image.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gameAudioVideo");
	ros::NodeHandle nodeHandle;
	ros::Publisher videoPublisher = nodeHandle.advertise<sensor_msgs::Image>("gameVideo", 5);
	while (ros::ok())
	{
		ros::spinOnce();
	}
}
