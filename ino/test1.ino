/*
**
**A0-encoder_green
**A1-voltage
**
**/
  
#include <ros.h>  
#include <std_msgs/String.h>  
#include <stdio.h>

//ros_node
ros::NodeHandle nh;  
//ros::NodeHandle sub_nh;  
std_msgs::String voltage_msg;
std_msgs::String encoder_msg;
ros::Publisher voltage_chatter("voltage_reader", &voltage_msg);   
ros::Publisher encoder_chatter("encoder_reader", &encoder_msg);
//grey-5v  purple-GND  green-D0
unsigned long encoderRight_count = 0, encoderLeft_count = 0;
double d = 7.0; //the diameter of the wheels
double pi = 3.1415926;

int count;

void beginCalculateDistance(const std_msgs::String& msg){  
  encoderLeft_count = 0;
  encoderRight_count = 0;
} 

void endCalculateDistance(const std_msgs::String& msg){  
  encoderCheck();
} 

void getVolatge(const std_msgs::String& msg){  
  voltageCheck();
} 

ros::Subscriber<std_msgs::String> encoder_begin("encoder_begin", &beginCalculateDistance);
ros::Subscriber<std_msgs::String> encoder_end("encoder_end", &endCalculateDistance);
ros::Subscriber<std_msgs::String> voltage_get("voltage_get", &getVolatge);


char* dtostr(char *str, double d)
{
  sprintf(str, "%f", d);
  return str;
}

void voltageCheck()
{
  char message[5] = "1.23";  
  
  double sensorValue = analogRead(A1);
  sensorValue /= 1023;
  sensorValue *= 5;
  
  dtostrf(sensorValue, 1, 2, message);
  voltage_msg.data = message;  
  voltage_chatter.publish( &voltage_msg );  
  nh.spinOnce();  
}

void encoderCheck()
{
  char message[30] = "";  
  
  if(encoderLeft_count > 0)
    encoderLeft_count -= 1;
  //double speedValue_left = d*pi*double(encoderLeft_count)/20.0/2/2; 
  double distance_left = d*pi*double(encoderLeft_count)/20.0/2; 
  if(encoderRight_count > 0)
    encoderRight_count -= 1;
  //double speedValue_right = d*pi*double(encoderRight_count)/20.0/2/2; 
  double distance_right = d*pi*double(encoderRight_count)/20.0/2; 
  
  char message_left[10] = "";  
  //dtostrf(speedValue_left, 2, 2, message_left);
  dtostrf(distance_left, 2, 2, message_left);
  char message_right[10] = "";
  //dtostrf(speedValue_right, 2, 2, message_right);
  dtostrf(distance_right, 2, 2, message_right);
  
  strcat(message, message_left);
  strcat(message," ");
  strcat(message, message_right);
  
  encoder_msg.data = message;  
  encoder_chatter.publish( &encoder_msg );  
  nh.spinOnce();  
  
  encoderRight_count = 0; 
  encoderLeft_count = 0;
}

void setup()  
{  
  Serial.begin(57600);
  nh.initNode();  
  //sub_nh.initNode();
  nh.advertise(voltage_chatter);
  nh.advertise(encoder_chatter);  
  nh.subscribe(encoder_begin);
  nh.subscribe(encoder_end);
  nh.subscribe(voltage_get); 
  
  count=0;
  
  attachInterrupt(0,D2ISR,CHANGE);
  attachInterrupt(1,D3ISR,CHANGE);
  
}

void D2ISR()
{
  encoderLeft_count++;
}

void D3ISR()
{
  encoderRight_count++;
}

  
void loop()  
{  
  /*
  delay(1000);
  count++;
  
  if(true)
    voltageCheck();
    
  if(count%2==0)
  {
    count=0;
    encoderCheck();
  }*/
  
  nh.spinOnce();
  delay(1); 
}  

