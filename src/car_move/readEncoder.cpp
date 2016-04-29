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
	if(speed_count > 2){
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
	ros::Subscriber vol_sub = n.subscribe("voltage_reader", 1000, volCallback);

	// RaspiRobot::getInstance()->forwardBySpeed(50);
	// while(!speed_read) {
	// 	ros::spinOnce();
	// }

	// RaspiRobot::getInstance()->forwardBySpeed(100);
	// speed_read = false; 
	// speed_count = 0;
	// while(!speed_read) {
	// 	ros::spinOnce();
	// }
	// RaspiRobot::getInstance()->turnLeft(90);
	// delay(5000);
	// RaspiRobot::getInstance()->turnLeft(45);
	// delay(5000);
	// RaspiRobot::getInstance()->turnRight(90);
	// delay(5000);
	// RaspiRobot::getInstance()->turnRight(45);
	// delay(5000);
	RaspiRobot::getInstance()->rotate_clockwise(90);

	ROS_INFO("I heard: [%s]", "shutdown");

	RaspiRobot::getInstance()->stop();
	// RaspiRobot::getInstance()->forwardByTimeAndSpeed(20, 50);

	// RaspiRobot::getInstance()->forwardByTimeAndSpeed(20, 100);

	// RaspiRobot::getInstance()->stop();

	// ros::spin();

	return 0;
}