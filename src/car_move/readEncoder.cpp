#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

bool speed_read = false;
int speed_count = 0;

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("encoder: [%s]", msg->data.c_str());

	speed_count++;
	if(speed_count > 3){
		speed_read = true;
	}
}

void volCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("voltage: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){

	ros::init(argc, argv, "encoder_reader");

	RaspiRobot::init();

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("encoder_reader", 1000, moveCallback);
	ros::Subscriber vol_sub = n.subscribe("voltage_reader", 5000, volCallback);
	
	while(!speed_read) {
		RaspiRobot::getInstance()->forwardBySpeed(70);
		ros::spinOnce();
	}

	speed_read = false; speed_count = 0;
	while(!speed_read) {
		RaspiRobot::getInstance()->forwardBySpeed(50);
		ros::spinOnce();
	}

	ROS_INFO("I heard: [%s]", "shutdown");

	RaspiRobot::getInstance()->stop();

	return 0;
}