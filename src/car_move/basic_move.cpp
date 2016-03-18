#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void moveCallback(const std_msgs::String::ConstPtr& msg){

	// RaspiRobot::getInstance()->forwardBySpeed(40);
	
	// delay(5000);

	// RaspiRobot::getInstance()->forwardBySpeed(100);

	// delay(5000);

	float f = RaspiRobot::getInstance()->getDistance();

	ROS_INFO("I heard: [%s %f]", msg->data.c_str(), f);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "basic_move");

	RaspiRobot::init();

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("move_chatter", 1000, moveCallback);

	ros::spin();

	return 0;
}
