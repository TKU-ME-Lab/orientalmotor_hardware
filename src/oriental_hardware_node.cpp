#include <ros/ros.h>
#include <iostream>
#define _DEBUG 1
#include "coriental_motor.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "oriental_hardware_node");
  ros::NodeHandle nh;

  COrientalMotor motor("/dev/U2D_AL01QGU9", 115200, 'E', 8, 1, 1);

  ros::Duration rate(1.5);

  while(ros::ok())
  {
    motor.read();
    rate.sleep();
  }

  return 0;
}