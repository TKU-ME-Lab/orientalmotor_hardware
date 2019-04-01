#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#define _DEBUG 1
#include "coriental_motor.h"

float position_data = 0;
float velocity_data = 10.0;
bool start = false;


void position_data_callback(const std_msgs::Float32ConstPtr &msg)
{
  ROS_INFO("Received [%f]" , msg->data);
  position_data = msg->data;
}

void velocity_data_callback(const std_msgs::Float32ConstPtr &msg)
{
  ROS_INFO("Received [%f]" , msg->data);
  velocity_data = msg->data;
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

  ros::Subscriber subscriber_position_cmd = nh.subscribe("/position_command", 10, position_data_callback);
  ros::Subscriber subscriber_velocity_cmd = nh.subscribe("/velocity_command", 10, velocity_data_callback);
  ros::Subscriber subscriber_start = nh.subscribe("/start", 10, start_callback);

  ros::Rate rate(1);

  while(ros::ok())
  {
    motor.read();
    if (start)
    {
      motor.move_absolute_position(position_data, velocity_data);
      start = false;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}