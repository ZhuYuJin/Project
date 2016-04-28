#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

bool speed_read = false;
int speed_count = 0;

int main(int argc, char **argv){

	ros::init(argc, argv, "encoder_reader");

	RaspiRobot::init();

	ros::NodeHandle n;

	//RaspiRobot::getInstance()->forwardByTimeAndSpeed(20, 50);

	RaspiRobot::getInstance()->forwardByTimeAndSpeed1(20);

	RaspiRobot::getInstance()->forwardByTimeAndSpeed1(20, 100);

	RaspiRobot::getInstance()->stop();

	return 0;
}