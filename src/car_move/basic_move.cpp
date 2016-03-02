#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	RaspiRobot::getInstance()->forwardBySpeed(40);
	
	delay(5000);

	RaspiRobot::getInstance()->forwardBySpeed(100);

	delay(5000);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "basic_move");

	RaspiRobot::init();

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("move_chatter", 1000, moveCallback);

	ros::spin();

	return 0;
}