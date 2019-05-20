#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <boost/scoped_ptr.hpp>
#include <std_srvs/SetBool.h>
#include <orientalmotor_hardware_msgs/SetProfileVelocityService.h>

#include "coriental_actuator.h"

class COrientalHardware: public hardware_interface::RobotHW
{
  typedef std::map<std::string, COrientalActuator*> OrientalMotorMap;

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  ros::ServiceServer m_ServiceServer_Autohome;
  ros::ServiceServer m_ServiceServer_SetProfileVelocity;

  OrientalMotorMap m_OrientalMotorMap;

  hardware_interface::JointStateInterface m_jsi;
  hardware_interface::PositionJointInterface m_pji;

  bool AutohomeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool SetProfileVelocityCallback(orientalmotor_hardware_msgs::SetProfileVelocityService::Request &req, orientalmotor_hardware_msgs::SetProfileVelocityService::Response &res);

public:
  COrientalHardware(ros::NodeHandle&, ros::NodeHandle&, std::vector<std::string>);

  bool init();

  void read();
  void write();
};
