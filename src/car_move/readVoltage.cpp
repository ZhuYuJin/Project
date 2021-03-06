// #include "RobotController.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){

	ros::init(argc, argv, "voltage_reader");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("voltage_reader", 1000, moveCallback);

	ros::spin();

	return 0;
}