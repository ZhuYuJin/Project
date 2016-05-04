#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void readVoltage(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard voltage: [%s]", msg->data.c_str());
}

void readEncoder(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard encoder: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test");

  ros::NodeHandle n;

  ros::Publisher voltage_get = n.advertise<std_msgs::String>("voltage_get", 1000);
  ros::Publisher encoder_begin = n.advertise<std_msgs::String>("encoder_begin", 1000);
  ros::Publisher encoder_end = n.advertise<std_msgs::String>("encoder_end", 1000);
  ros::Subscriber voltage_chatter = n.subscribe("voltage_reader", 1000, readVoltage);
  ros::Subscriber encoder_chatter = n.subscribe("encoder_reader", 1000, readEncoder);

  ros::Rate loop_rate(1);

  std_msgs::String msg;
  msg.data = "123";

  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();
  loop_rate.sleep();

  voltage_get.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();

  voltage_get.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();

  encoder_begin.publish(msg);
  for(int i = 0; i < 10; i++)
    loop_rate.sleep();
  encoder_end.publish(msg);
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();




  // int count = 0;
  // while (ros::ok()){
  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "Get distance" << count;
  //   msg.data = ss.str();

  //   // ROS_INFO("%s", msg.data.c_str());

  //   test_pub.publish(msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }

  return 0;
}
