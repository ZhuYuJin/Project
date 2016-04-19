#include "RobotBasicController.h"
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>  

pthread_mutex_t qlock = PTHREAD_MUTEX_INITIALIZER;    //初始构造锁  

void moveCallback(const std_msgs::String::ConstPtr& msg){

	// RaspiRobot::getInstance()->forwardBySpeed(40);
	
	// delay(5000);

	// RaspiRobot::getInstance()->forwardBySpeed(100);

	// delay(5000);

	pthread_mutex_lock(&qlock);

	double f = RaspiRobot::getInstance()->getDistance();

	ROS_INFO("I know the distance is: [%f]", f);

	if(f < 10.0){

		RaspiRobot::getInstance()->turnLeft(30.0, 50.0, 50.0, 20.0);

		RaspiRobot::getInstance()->turnRight(30.0, 50.0, 50.0, 20.0);
	
		RaspiRobot::getInstance()->forwardByTimeAndSpeed(5, 50.0);

		RaspiRobot::getInstance()->turnRight(30.0, 50.0, 50.0, 20.0);

		RaspiRobot::getInstance()->turnLeft(30.0, 50.0, 50.0, 20.0);
	}

	pthread_mutex_unlock(&qlock); 
}

int main(int argc, char **argv){

	ros::init(argc, argv, "basic_move");

	RaspiRobot::init();

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("move_chatter", 1000, moveCallback);

	ros::spin();

	return 0;
}
