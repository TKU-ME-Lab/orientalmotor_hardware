#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <iostream>
#define _DEBUG 1
#include "coriental_motor.h"

double data = 0;
bool start = false;


void data_callback(const std_msgs::Float64ConstPtr &msg)
{
  ROS_INFO("Received [%f]" , msg->data);
  data = msg->data;
}

void start_callback(const std_msgs::BoolConstPtr &msg)
{
  ROS_INFO("Get Start Signal");
  start = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oriental_hardware_node");
  ros::NodeHandle nh;

  COrientalMotor motor("/dev/U2D_AL01QGU9", 115200, 'E', 8, 1, 1);

  ros::Subscriber subscriber_cmd = nh.subscribe("/command", 10, data_callback);
  ros::Subscriber subscriber_start = nh.subscribe("/start", 10, start_callback);

  ros::Duration rate(1.5);

  while(ros::ok())
  {
    motor.read();
    if (start)
    {
      motor.move_absolute_position(data);
      start = false;
    }
    rate.sleep();
  }

  return 0;
}