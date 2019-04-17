#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <boost/scoped_ptr.hpp>

#include "coriental_actuator.h"

class COrientalHardware: public hardware_interface::RobotHW
{
  typedef std::map<std::string, COrientalActuator*> OrientalMotorMap;

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  OrientalMotorMap m_OrientalMotorMap;

  hardware_interface::ActuatorStateInterface m_asi;
  hardware_interface::PositionActuatorInterface m_api;

  transmission_interface::RobotTransmissions m_robot_transmissions;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> m_transmission_loader;


public:
  COrientalHardware(ros::NodeHandle&, ros::NodeHandle&, std::vector<std::string>);

  bool init();

  void read();
  void write();
};
