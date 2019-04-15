#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "coriental_actuator.h"

class COrientalHardware: public hardware_interface::RobotHW
{
  typedef std::map<std::string, COrientalActuator*> OrientalMotorMap;

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  OrientalMotorMap m_OrientalMotorMap;

  hardware_interface::JointStateInterface m_jsi;
  hardware_interface::PositionJointInterface m_jpi;

public:
  COrientalHardware(ros::NodeHandle&, ros::NodeHandle&, std::vector<std::string>);

  bool init();

  void read();
  void write();
};
