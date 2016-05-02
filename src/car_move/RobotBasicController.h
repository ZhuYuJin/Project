
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>

# define M_PI       	3.14159265358979323846

#define uchar 			unsigned char

#define HIGH			1
#define LOW				0

#define LEFT_IN_PIN		7
#define LEFT_OUT_PIN	0
#define RIGHT_IN_PIN	2
#define RIGHT_OUT_PIN	3
#define	LEFT_EN_PWM		1
#define RIGHT_EN_PWM	4
#define TRIGGER_PIN		21
#define ECHO_PIN		22
#define LASER_PIN		23

//speed of the car: 50-34cm/s 70-46cm/s
static int FULL_SPEED = 90;
static int FULL_SPEED_EN = 100;
static int WHEEL_BASE = 20;

using namespace std;

class RaspiRobot
{
	private:
		static RaspiRobot *instance;
		int direction;
		RaspiRobot();
		void setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn);
		void setMotors1(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn, uchar last_time);
		float getDistance(float minDistance,float maxDistance,int count,int maxLoop);
	public:
		static bool init();
		static RaspiRobot *getInstance();
		void stop();
		void forwardBySpeed(int speed);
		void forwardByTimeAndSpeed(float sec, int speed);
		void forwardByTimeAndSpeed1(float sec, int speed);
		void reverseBySpeed(int speed);
		void reverseByTimeAndSpeed(float sec, int speed);
		void turnLeft(float degree, float speed, float speed_t, float wheelbase);
		void turnRight(float degree, float speed, float speed_t, float wheelbase);
		void rotate_clockwise(float degree, int speed, float wheelbase);
		void rotate_anticlockwise(float degree, int speed, float wheelbase);
		float getDistance();
		unsigned int getVoltage();
		bool checkNavigationLaser();
};

RaspiRobot *RaspiRobot::getInstance()
{
	if(instance==NULL)
		instance=new RaspiRobot();
	return instance;
}

RaspiRobot::RaspiRobot()
{
	
}

RaspiRobot *RaspiRobot::instance=NULL;

bool RaspiRobot::init()
{
	if(wiringPiSetup() < 0){
		printf("setup wiringPi failed !");
		return false;
	}
	
	pinMode(LEFT_IN_PIN, OUTPUT);
	pinMode(LEFT_OUT_PIN, OUTPUT);
	pinMode(RIGHT_IN_PIN, OUTPUT);
	pinMode(RIGHT_OUT_PIN, OUTPUT);
	softPwmCreate(LEFT_EN_PWM, 0, 100);
	softPwmCreate(RIGHT_EN_PWM, 0, 100);
	pinMode(TRIGGER_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	pinMode(LASER_PIN, INPUT);
	return true;
}

void RaspiRobot::setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn)
{
	digitalWrite(LEFT_IN_PIN, leftIn);
	digitalWrite(LEFT_OUT_PIN, leftOut);
	digitalWrite(RIGHT_IN_PIN, rightIn);
	digitalWrite(RIGHT_OUT_PIN, rightOut);
	if(LEFT_IN_PIN == 0 && LEFT_OUT_PIN == 1)
		softPwmWrite(LEFT_EN_PWM, leftEn-40);
	else if(LEFT_IN_PIN == 1 && LEFT_OUT_PIN == 0)
		softPwmWrite(LEFT_EN_PWM, leftEn-5);
	else
		softPwmWrite(LEFT_EN_PWM, leftEn);
	softPwmWrite(RIGHT_EN_PWM, rightEn);
}

// void RaspiRobot::setMotors1(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn, uchar last_time)
// {
// 	pinMode(LEFT_EN_PWM, OUTPUT);
// 	pinMode(RIGHT_EN_PWM, OUTPUT);

// 	digitalWrite(LEFT_IN_PIN, leftIn);
// 	digitalWrite(LEFT_OUT_PIN, leftOut);
// 	digitalWrite(RIGHT_IN_PIN, rightIn);
// 	digitalWrite(RIGHT_OUT_PIN, rightOut);
// 	if(leftEn == 50){
// 		for(int i = 0; i < last_time; i++){
// 			for(int i = 0; i < 20; i++){
// 				digitalWrite(LEFT_EN_PWM, 0);
// 				digitalWrite(RIGHT_EN_PWM, 1);
// 				delay(10);
// 				digitalWrite(LEFT_EN_PWM, 1);
// 				digitalWrite(RIGHT_EN_PWM, 1);
// 				delay(20);
// 				digitalWrite(LEFT_EN_PWM, 0);
// 				digitalWrite(RIGHT_EN_PWM, 0);
// 				delay(20);
// 			}
// 		}
// 	}else if(leftEn == 100){
// 		for(int i = 0; i < last_time; i++){
// 			for(int i = 0; i < 20; i++){
// 				digitalWrite(LEFT_EN_PWM, 0);
// 				digitalWrite(RIGHT_EN_PWM, 1);
// 				delay(10);
// 				digitalWrite(LEFT_EN_PWM, 1);
// 				digitalWrite(RIGHT_EN_PWM, 1);
// 				delay(35);
// 				digitalWrite(LEFT_EN_PWM, 0);
// 				digitalWrite(RIGHT_EN_PWM, 0);
// 				delay(5);
// 			}
// 		}
// 	}else if(leftEn == 0){
// 		digitalWrite(LEFT_EN_PWM, 0);
// 		digitalWrite(RIGHT_EN_PWM, 0);
// 	}
// }

// void RaspiRobot::forwardByTimeAndSpeed1(float sec, int speed = 50)
// {
// 	setMotors1(0,1,0,1,speed,speed, sec);
// }

void RaspiRobot::stop()
{
	setMotors(0,0,0,0,0,0);
}

void RaspiRobot::forwardBySpeed(int speed)
{
	setMotors(0,1,0,1,speed,speed);
}

void RaspiRobot::forwardByTimeAndSpeed(float sec, int speed = 50)
{
	setMotors(0,1,0,1,speed,speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::reverseBySpeed(int speed)
{
	setMotors(1,0,1,0,speed,speed);
}

void RaspiRobot::reverseByTimeAndSpeed(float sec, int speed = 50)
{
	setMotors(1,0,1,0,speed,speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

//degree = 30, speed = 40, speed_t = 20, wheelbase = 15.2
void RaspiRobot::turnLeft(float degree, float speed = 0, float speed_t = FULL_SPEED, float wheelbase = WHEEL_BASE)
{
	// float r = (speed * wheelbase) / speed_t;//distance of the left wheel from the circle center
	// float d_r = (((wheelbase + r) * degree) / 180.0) * M_PI;
	// float d_l = ((r * degree) / 180.0) * M_PI;
	// float t1 = d_r / (speed + speed_t);
	// float t2 = d_l / speed;
	// float sec = (t1 + t2) / 2;
	// // setMotors(0,1,0,1,(int)speed,(int)(speed+speed_t));
	// setMotors(0,1,0,1,0,100);
	// if(sec>0)
	// {
	// 	delay((int)(sec*1000));
	// 	stop();
	// }

	float r = wheelbase;
	float sec = M_PI * r * degree / 180.0 / speed_t;
	sec *= 3;
	setMotors(0,1,0,1,0,100);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

//degree = 30, speed = 40, speed_t = 20, wheelbase = 15.2
void RaspiRobot::turnRight(float degree, float speed = 0, float speed_t = FULL_SPEED, float wheelbase = WHEEL_BASE)
{
	// float r = (speed * wheelbase) / speed_t;//distance of the right wheel from the circle center
	// float d_l = (((wheelbase + r) * degree) / 180.0) * M_PI;
	// float d_r = ((r * degree) / 180.0) * M_PI;
	// float t1 = d_l / (speed + speed_t);
	// float t2 = d_r / speed;
	// float sec = (t1 + t2) / 2;
	// setMotors(0,1,0,1,100,0);
	// if(sec>0)
	// {
	// 	delay((int)(sec*1000));
	// 	stop();
	// }

	float r = wheelbase;
	float sec = M_PI * r * degree / 180.0 / speed_t;
	sec *= 2;
	setMotors(0,1,0,1,100,0);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::rotate_clockwise(float degree, int speed = FULL_SPEED, float wheelbase = 20.0)
{
	float d = (degree / 360.0) * wheelbase * M_PI;
	float sec = d / speed;
	sec *= 2.5;
	// setMotors(0,1,1,0,(int)speed,(int)speed);
	setMotors(0,1,1,0,100,100);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::rotate_anticlockwise(float degree, int speed = FULL_SPEED, float wheelbase = 20.0)
{
	float d = (degree / 360.0) * wheelbase * M_PI;
	float sec = d / speed;
	sec *= 2.5;
	// setMotors(0,1,1,0,(int)speed,(int)speed);
	setMotors(1,0,0,1,100,100);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

float RaspiRobot::getDistance(float minDistance,float maxDistance,int count,int maxLoop)
{
	int successfulCount=0;
	float totalDistance=0;
	struct timeval t1,t2;
	
	while(maxLoop--)
	{
		digitalWrite(TRIGGER_PIN,LOW);
		delayMicroseconds(2);
		digitalWrite(TRIGGER_PIN,HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIGGER_PIN,LOW);
		
		int countdown=2000000;
		while(digitalRead(ECHO_PIN)!=HIGH&&countdown>0)
		{
			countdown--;
			//Without pulseIn() function on Arduino, so we can only wait here
		}
		if(countdown==0)
			continue;
		gettimeofday(&t1,NULL);

		countdown=5000000;
		while(digitalRead(ECHO_PIN)!=LOW&&countdown>0)
		{
			countdown--;
			//Without pulseIn() function on Arduino, so we can only wait here
		}
		if(countdown==0)
			continue;
		gettimeofday(&t2,NULL);
		
		long startMicrosecond=t1.tv_sec*1000000+t1.tv_usec;
		long endMicrosecond=t2.tv_sec*1000000+t2.tv_usec;
		float distance=(endMicrosecond-startMicrosecond)/58.8235;
		
		if(distance<minDistance||distance>maxDistance)
			continue;
		successfulCount++;
		totalDistance+=distance;
		
		if(successfulCount==count)
			return totalDistance/count;
	}
	return -1;
}

float RaspiRobot::getDistance()
{
	return getDistance(2.0,450.0,10,20);
}

bool RaspiRobot::checkNavigationLaser()
{
	int laser = digitalRead(LASER_PIN);
	return laser;
}