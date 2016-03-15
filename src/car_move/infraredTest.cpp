#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define uchar 			unsigned char

#define HIGH			1
#define LOW				0

#define INFRARED_DO		12
#define LED1			13

int main(int argc, char **argv){

	ros::init(argc, argv, "infraredTest");

	if(wiringPiSetup()<0)
		return false;

	pinMode(INFRARED_DO, INPUT);
	pinMode(LED1, OUTPUT);

	ROS_INFO("I heard: infrared");

	while(true){
		digitalWrite(LED1, digitalRead(INFRARED_DO));
	}

	return 0;
}