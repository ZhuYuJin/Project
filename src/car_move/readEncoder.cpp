#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	ros::spinOnce();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "encoder_reader");

	RaspiRobot::init();

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("encoder_reader", 1000, moveCallback);

	ros::spinOnce();

	RaspiRobot::getInstance()->forwardBySpeed(100);

	delay(20000);

	RaspiRobot::getInstance()->forwardBySpeed(50);

	delay(20000);

	return 0;
}