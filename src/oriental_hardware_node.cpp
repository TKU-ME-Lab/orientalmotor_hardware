#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <vector>

#include <coriental_hardware.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oriental_hardware_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //COrientalActuator motor("/dev/U2D_AL01QGU9", 115200, 'E', 8, 1, 1);
  std::vector<std::string> actuator_names;
  for (int i = 0; i < argc-1; ++i){
      actuator_names.push_back(argv[i+1]);
  }

  COrientalHardware robot(nh, pnh, actuator_names);
  controller_manager::ControllerManager CM(&robot, nh); 

  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (!robot.init())
  {
    ROS_FATAL("Failed to initializer actuators");
    return 0;
  }

  ros::Rate rate(1);
  ros::Time last = ros::Time::now();

  while(ros::ok())
  {
    robot.read();
    ros::Time now = ros::Time::now();
    CM.update(now, now-last);
    robot.write();
    last = now;
    rate.sleep();
  }

  return 0;
}