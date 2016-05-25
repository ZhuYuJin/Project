#include "RobotBasicController.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h> 
#include <math.h>
#include <pthread.h>  
#include <sys/time.h>

#define PI 3.14159262653589793238462643383279

#define BIG_RIGHT 	4
#define RIGHT 		3
#define MIDDLE 		2
#define BIG_LEFT 	1
#define LEFT 		0

#define AREA_CLOSE_DISTANCE 	50.0
#define AREA_FAR_DISTANCE		120.0
#define AREA_CLOSE 				1
#define AREA_MIDDLE				2
#define AREA_FAR				3
#define AREA_NONE				0

using namespace std;

/* barcode */
bool barcode_exist = false;
float barcode_distance;

double ang_r = 59, ang_c = 44.332; //ang_r->maximum horizontal angle    ang_c->maximum vertical angle
int row, col; //row->mximum horizontal pixel number    col->maximum vertical pixel number
int xc1, yc1, xc2, yc2;	//positions of feature points in the picture(described by pixel number)

double z, d; //d->edge of barcode	z->distance of barcode(to be solved)
double fix_number = 1.0508;

/* infrared */
bool infrared_exist = false;
float infrared_distance;

/* region */
int region;
int sideFromBarcode = -1;

/*data from Arduino*/
double voltage = -1;
double encoder_distance = -1;

int getRegionFromCam(){
	barcode_distance = -1;
	sideFromBarcode = -1;
	barcode_exist = false;

	delay(1000);
	ros::spinOnce();
	delay(1000);
	ros::spinOnce();
	delay(1000);
	ros::spinOnce();

	float degree = 0.0;
	float unit_degree = 6.0;
	while(!barcode_exist && degree < 360.0){
		if(barcode_distance != -1){
			degree += unit_degree/3; //data amendment
			RaspiRobot::getInstance()->rotate_clockwise(unit_degree);
		}else{
			degree += unit_degree*2/3; //data amendment
			RaspiRobot::getInstance()->rotate_clockwise(unit_degree*2);
		}
		delay(1000);
		ros::spinOnce();
		delay(1000);
	}

	barcode_exist = false;

	if(degree <= 360.0){
		if(barcode_distance > AREA_FAR_DISTANCE){
			return AREA_FAR;
		}else if(barcode_distance > AREA_CLOSE_DISTANCE){
			return AREA_MIDDLE;
		}else{
			return AREA_CLOSE;
		}
	}else{
		return AREA_NONE;
	}

}

int getRegionFromCam_l(){
	delay(1000);
	ros::spinOnce();

	float degree = 0.0;
	float unit_degree = 6.0;
	while(!barcode_exist && degree < 360.0){
		if(encoder_distance != -1){
			degree += unit_degree/3; //data amendment
			RaspiRobot::getInstance()->rotate_anticlockwise(unit_degree);
		}else{
			degree += unit_degree*2/3; //data amendment
			RaspiRobot::getInstance()->rotate_anticlockwise(unit_degree*2);
		}
		delay(1000);
		ros::spinOnce();
		delay(1000);
	}

	barcode_exist = false;

	if(degree <= 360.0){
		if(barcode_distance > AREA_FAR_DISTANCE){
			return AREA_FAR;
		}else if(barcode_distance > AREA_CLOSE_DISTANCE){
			return AREA_MIDDLE;
		}else{
			return AREA_CLOSE;
		}
	}else{
		return AREA_NONE;
	}

}

float getDistance(int x_mid, int y_mid, int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, double distance){
	row = 640;
	col = 480;
	z = 0.0;
	d = distance; //distance between middle point and top-left point

	double k_mid, p_mid, k1, p1, k2, p2, k3, p3, k4, p4;
	xc1 = x_mid;
	yc1 = y_mid;
	k_mid = (2*xc1-row) * tan(double(ang_r)/double(360)*PI) / row;
	p_mid = (2*yc1-col) * tan(double(ang_c)/double(360)*PI) / col;

	xc2 = x1;
	yc2 = y1;
	k1 = (2*xc2-row) * tan(double(ang_r)/double(360)*PI) / row;
	p1 = (2*yc2-col) * tan(double(ang_c)/double(360)*PI) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k1),2) + pow((p_mid-p1),2)));

	xc2 = x2;
	yc2 = y2;
	k2 = (2*xc2-row) * tan(double(ang_r)/double(360)*PI) / row;
	p2 = (2*yc2-col) * tan(double(ang_c)/double(360)*PI) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k2),2) + pow((p_mid-p2),2)));

	xc2 = x3;
	yc2 = y3;
	k3 = (2*xc2-row) * tan(double(ang_r)/double(360)*PI) / row;
	p3 = (2*yc2-col) * tan(double(ang_c)/double(360)*PI) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k3),2) + pow((p_mid-p3),2)));

	xc2 = x4;
	yc2 = y4;
	k4 = (2*xc2-row) * tan(double(ang_r)/double(360)*PI) / row;
	p4 = (2*yc2-col) * tan(double(ang_c)/double(360)*PI) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k4),2) + pow((p_mid-p4),2)));

	z = z/4;

	ROS_INFO("Distance from cam: [%f]", z);

	return z;
}

void barcodeCheck(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	double mid_x, mid_y, x_min, x_max, y_min, y_max, x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4; //infos of barcode
	string str; // message of barcode

	stringstream ss(msg->data.c_str());
	string temp;
	for(int i = 0; i < 15; i++){
		ss >> temp;
		if(i == 0) mid_x = atof(temp.c_str());
		if(i == 1) mid_y = atof(temp.c_str());
		if(i == 2) x_min = atof(temp.c_str());
		if(i == 3) x_max = atof(temp.c_str());
		if(i == 4) y_min = atof(temp.c_str());
		if(i == 5) y_max = atof(temp.c_str());
		if(i == 6) x_1 = atof(temp.c_str());
		if(i == 7) y_1 = atof(temp.c_str());
		if(i == 8) x_2 = atof(temp.c_str());
		if(i == 9) y_2 = atof(temp.c_str());
		if(i == 10) x_3 = atof(temp.c_str());
		if(i == 11) y_3 = atof(temp.c_str());
		if(i == 12) x_4 = atof(temp.c_str());
		if(i == 13) y_4 = atof(temp.c_str());
		if(i == 14) str = temp.c_str();
	}
	
	barcode_distance = getDistance(mid_x, mid_y, x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4, 9.825);

	/*
		X2       	X1
			X_mid
		X3       	X4
	*/
	if(sideFromBarcode == -1){
		int y_l = abs(y_2-y_1);
		int y_r = abs(y_3-y_4);
		if(y_l > y_r){
			sideFromBarcode = LEFT;
		}else if(y_l == y_r){
			sideFromBarcode = MIDDLE;
		}else{
			sideFromBarcode = RIGHT;
		}
		// if(barcode_distance > AREA_CLOSE_DISTANCE){
		// 	int y_diff = abs(y_l-y_r);
		// 	double y_threshold = 20.0-(barcode_distance-60.0)/10.0;
		// 	if(y_diff > y_threshold){
		// 		if(y_l > y_r){
		// 			sideFromBarcode = BIG_LEFT;
		// 		}else{
		// 			sideFromBarcode = BIG_RIGHT;
		// 		}
		// 	}
		// }
	}

	if(mid_x > 290 && mid_x < 350)
		barcode_exist = true;
}

void readVoltage(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard voltage: [%s]", msg->data.c_str());
	voltage = atof(msg->data.c_str());
}

void readEncoder(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard encoder: [%s]", msg->data.c_str());
	double distance_left, distance_right;
	stringstream ss(msg->data.c_str());
	string temp;
	ss >> temp;
	distance_left = atof(temp.c_str());
	ss >> temp;
	distance_right = atof(temp.c_str());
	encoder_distance = (distance_left+distance_right)/2;
}

// void infraredCheck(const std_msgs::String::ConstPtr& msg){
// 	ROS_INFO("I heard: [%s]", msg->data.c_str());
// 	infrared_exist = true;
// }

bool obstacleDetected(){
	//getBarcodeInfo
	ros::spinOnce();
	delay(1500);

	bool is_block = false;

	//get the distance of the block
	float distance = 0.0;
	float temp = 0.0;
	float distance_max = 0.0;
	float distance_min = 9999.0;
	for(int i = 0 ; i < 7; i++){
		temp = RaspiRobot::getInstance()->getDistance();
		if(temp > distance_max){
			distance_max = temp;
		}
		if(temp < distance_min){
			distance_min = temp;
		}
		distance += temp;
	}
	distance -= distance_min;
	distance -= distance_max;
	distance /= 5;
	ROS_INFO("Distance from echo: [%f]", distance);

	//if distance is smaller than 15 and bigger than 0. 
	//Barcode does not exist at the same time. 
	//We have the conclusion that block exists.
	if(distance <= 30.0 && distance >= 0 && !barcode_exist){
		is_block = true;
	}

	barcode_exist = false;
	return is_block;
}

void avoidObstacle(){
	// bool is_block = obstacleDetected();
	// if(is_block){
		RaspiRobot::getInstance()->rotate_anticlockwise(45);
		RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.4, FULL_SPEED_EN);
		RaspiRobot::getInstance()->rotate_clockwise(45);
		// avoidObstacle();
		RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.5, FULL_SPEED_EN);
		RaspiRobot::getInstance()->rotate_clockwise(45);
		RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.4, FULL_SPEED_EN);
		RaspiRobot::getInstance()->rotate_anticlockwise(45);
	// }
}

bool searchNavigationSignal()
{
	bool laser = RaspiRobot::getInstance()->checkNavigationLaser();
	struct timeval t1, t2;
	long startMicrosecond, endMicrosecond;
	gettimeofday(&t1,NULL);
	startMicrosecond = t1.tv_sec*1000000+t1.tv_usec;
	RaspiRobot::getInstance()->forwardBySpeed(FULL_SPEED_EN);
	while(laser != 0){
		laser = RaspiRobot::getInstance()->checkNavigationLaser();
		gettimeofday(&t2,NULL);
		endMicrosecond = t2.tv_sec*1000000+t2.tv_usec;
		if((endMicrosecond-startMicrosecond) > 10000000.0){
			return false;
		}
	}
	return true;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "AutoChargeRobot");

	RaspiRobot::init(); //init robot controller

	ros::NodeHandle n;
	ros::Subscriber sub_bar = n.subscribe("barcode", 1000, barcodeCheck);
	/*Arduino Node*/
	// ros::Publisher voltage_get = n.advertise<std_msgs::String>("voltage_get", 1000);
 //  	ros::Publisher encoder_begin = n.advertise<std_msgs::String>("encoder_begin", 1000);
 //  	ros::Publisher encoder_end = n.advertise<std_msgs::String>("encoder_end", 1000);
 //  	ros::Subscriber voltage_chatter = n.subscribe("voltage_reader", 1000, readVoltage);
	// ros::Subscriber encoder_chatter = n.subscribe("encoder_reader", 1000, readEncoder);

	delay(15000);//time for setup

// 	ros::Rate loop_rate(1);
// 	//get Voltage
// 	std_msgs::String msg;
//   	msg.data = "getVoltage";
// 	voltage_get.publish(msg);
// 	ros::spinOnce();
// 	loop_rate.sleep();

// //section 2 avoid obstacle
// 	// bool have_avoid = false;

// 	// while(!have_avoid){
// 	// 	if(obstacleDetected()){
// 	// 		avoidObstacle();
// 	// 		have_avoid = true;
// 	// 	}else{
// 	// 		RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.2, FULL_SPEED_EN);
// 	// 	}
// 	// }	

// 	// RaspiRobot::getInstance()->stop();

// //section 3

	// sideFromBarcode = -1;
	// region = getRegionFromCam();

	// while(region == 0){
	// 	//if QR not found
	// 	//randomwalk
	// 	RaspiRobot::getInstance()->forwardByTimeAndSpeed(1, FULL_SPEED_EN);
	// 	sideFromBarcode = -1;
	// 	region = getRegionFromCam();
	// }

	bool docked = false;
	while(!docked){
		sideFromBarcode = -1;
		region = getRegionFromCam();
		while(region == 0){
			//if QR not found
			//randomwalk
			RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.5, FULL_SPEED_EN);
			sideFromBarcode = -1;
			region = getRegionFromCam();
		}

		if(region == AREA_FAR){
			RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.5, FULL_SPEED_EN);
		}else if(region == AREA_MIDDLE){
ROS_INFO("sideFromBarcode:%d", sideFromBarcode);
			if(sideFromBarcode == RIGHT){
				RaspiRobot::getInstance()->rotate_anticlockwise(30);
ROS_INFO("searchNavigationSignal");
				searchNavigationSignal();
ROS_INFO("getRegion again");
				sideFromBarcode = -1;
				region = getRegionFromCam();
				while(region == 0){
					RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.1, FULL_SPEED_EN);
					sideFromBarcode = -1;
					region = getRegionFromCam();
				}
			}else if(sideFromBarcode == LEFT){
				RaspiRobot::getInstance()->rotate_clockwise(30);
				searchNavigationSignal();
				sideFromBarcode = -1;
				region = getRegionFromCam_l();
				while(region == 0){
					RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.1, FULL_SPEED_EN);
					sideFromBarcode = -1;
					region = getRegionFromCam_l();
				}
			}else if(sideFromBarcode == BIG_RIGHT){
				RaspiRobot::getInstance()->rotate_anticlockwise(120);
				RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.5, FULL_SPEED_EN);
				RaspiRobot::getInstance()->rotate_clockwise(90);
				sideFromBarcode = -1;
				continue;
			}else if(sideFromBarcode == BIG_LEFT){
				RaspiRobot::getInstance()->rotate_clockwise(120);
				RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.5, FULL_SPEED_EN);
				RaspiRobot::getInstance()->rotate_anticlockwise(90);
				sideFromBarcode = -1;
				continue;
			}
			
			// //encoder begin
			// msg.data = "encoder begin";
			// encoder_begin.publish(msg);
			// ros::spinOnce();
			// loop_rate.sleep();

			float t = barcode_distance/FULL_SPEED;
			RaspiRobot::getInstance()->forwardByTimeAndSpeed(t, FULL_SPEED_EN);
		
			// //encoder end
			// msg.data = "encoder end";
			// encoder_end.publish(msg);
			// loop_rate.sleep();
			// ros::spinOnce();
			// loop_rate.sleep();

			// double distance_t = barcode_distance-encoder_distance;
			// if(distance_t > 0){
			// 	t = distance_t/FULL_SPEED;
			// 	RaspiRobot::getInstance()->forwardByTimeAndSpeed(t, FULL_SPEED_EN);
			// }

			docked = true;
			break;
		}else{
			if(sideFromBarcode == RIGHT){
				RaspiRobot::getInstance()->rotate_anticlockwise(120);
				RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.6, FULL_SPEED_EN);
			}else if(sideFromBarcode == LEFT){
				RaspiRobot::getInstance()->rotate_clockwise(120);
				RaspiRobot::getInstance()->forwardByTimeAndSpeed(0.6, FULL_SPEED_EN);
			}
		}
	}

	RaspiRobot::getInstance()->stop();

	return 0;
}