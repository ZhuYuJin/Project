#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define uchar 			unsigned char

#define HIGH			1
#define LOW				0

#define INFRARED_DO		12

int main(int argc, char **argv){

	ros::init(argc, argv, "basic_move");

	RaspiRobot::init();

	if(wiringPiSetup()<0)
		return false;

	pinMode(INFRARED_DO, OUTPUT);

	digitalWrite(INFRARED_DO, HIGH);
	delay(5000);
	digitalWrite(INFRARED_DO, LOW);
	delay(5000);
	digitalWrite(INFRARED_DO, HIGH);

	ROS_INFO("I heard: infrared");

	return 0;
}