
#include "ros/ros.h"
#include "std_msgs/String.h"

struct timeval t1, t2;
long startMicrosecond, endMicrosecond;

void barcodeCheck(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	gettimeofday(&t1,NULL);
	startMicrosecond=t1.tv_sec*1000000+t1.tv_usec;
	ROS_INFO("start:%ld", startMicrosecond);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "AutoChargeRobot");

	ros::NodeHandle n;
	ros::Subscriber sub_bar = n.subscribe("barcode", 1000, barcodeCheck);

	ros::Rate rate_loop(0.2);
	rate_loop.sleep();
	ros::spinOnce();
	rate_loop.sleep();
	ros::spinOnce();
	rate_loop.sleep();
	ros::spinOnce();

	gettimeofday(&t2,NULL);
	endMicrosecond=t2.tv_sec*1000000+t2.tv_usec;

	ROS_INFO("end:%ld", endMicrosecond);

	return 0;
}