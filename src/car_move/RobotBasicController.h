
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

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

using namespace std;

class RaspiRobot
{
	private:
		static RaspiRobot *instance;
		int direction;
		RaspiRobot();
		void setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn);

	public:
		static bool init();
		static RaspiRobot *getInstance();
		void stop();
		void forwardBySpeed(int speed);
		void forwardByTimeAndSpeed(float sec, int speed);
		void reverseBySpeed(int speed);
		void reverseByTimeAndSpeed(float sec, int speed);
		void turnLeft(float degree, float speed, float speed_t, float wheelbase);
		void turnRight(float degree, float speed, float speed_t, float wheelbase);
		void rotate_clockwise(float degree, int speed, float wheelbase);
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
	return true;
}

void RaspiRobot::setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn)
{
	digitalWrite(LEFT_IN_PIN, leftIn);
	digitalWrite(LEFT_OUT_PIN, leftOut);
	digitalWrite(RIGHT_IN_PIN, rightIn);
	digitalWrite(RIGHT_OUT_PIN, rightOut);
	softPwmWrite(LEFT_EN_PWM, leftEn);
	softPwmWrite(RIGHT_EN_PWM, rightEn);
}

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
void RaspiRobot::turnLeft(float degree, float speed, float speed_t, float wheelbase)
{
	float r = (speed * wheelbase) / speed_t;//distance of the left wheel from the circle center
	float d_r = (((wheelbase + r) * degree) / 180.0) * M_PI;
	float d_l = ((r * degree) / 180.0) * M_PI;
	float t1 = d_r / (speed + speed_t);
	float t2 = d_l / speed;
	float sec = (t1 + t2) / 2;
	setMotors(0,1,0,1,(int)speed,(int)(speed+speed_t));
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

//degree = 30, speed = 40, speed_t = 20, wheelbase = 15.2
void RaspiRobot::turnRight(float degree, float speed, float speed_t, float wheelbase)
{
	float r = (speed * wheelbase) / speed_t;//distance of the right wheel from the circle center
	float d_l = (((wheelbase + r) * degree) / 180.0) * M_PI;
	float d_r = ((r * degree) / 180.0) * M_PI;
	float t1 = d_l / (speed + speed_t);
	float t2 = d_r / speed;
	float sec = (t1 + t2) / 2;
	setMotors(0,1,0,1,(int)(speed+speed_t),(int)speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::rotate_clockwise(float degree, int speed, float wheelbase)
{
	float d = (degree / 360.0) * wheelbase * M_PI;
	float sec = d / speed;
	setMotors(0,1,1,0,(int)speed,(int)speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}