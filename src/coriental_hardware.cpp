#include "coriental_hardware.h"
#include <boost/foreach.hpp>

COrientalHardware::COrientalHardware(ros::NodeHandle& nh, ros::NodeHandle& private_nh, std::vector<std::string> motor_names):
  m_nh(nh), m_private_nh(private_nh)
{
  BOOST_FOREACH(const std::string motor_name, motor_names)
  {
    ros::NodeHandle motor_config(m_private_nh, motor_name);
    OrientalParammeter param;

    if (motor_config.getParam("port", param.port_name))
    {
      if (motor_config.getParam("baud_rate", param.baud_rate))
      {
        std::string parity;
        if (motor_config.getParam("parity", parity))
        {
          param.parity = parity.at(0);
          int data = 0;
          if (motor_config.getParam("data_bit", data))
          {
            param.data_bit = data;
            if (motor_config.getParam("stop_bit", data))
            {
              param.stop_bit = data;
              if (motor_config.getParam("id", data))
              {
                param.id = data;
                if (!motor_config.getParam("profile_velocity", param.profile_velocity))
                {
                  ROS_ERROR("Didn't have 'max_position' in config");
                  return;
                }

                COrientalActuator* Actuator = new COrientalActuator(param);
                m_OrientalMotorMap[motor_name] = Actuator;

              }
              else
              {
                ROS_ERROR("Didn't have 'id' in config");
                return;
              }
            }
            else
            {
              ROS_ERROR("Didn't have 'stop_bit' in config");
              return;
            }
          }
          else
          {
            ROS_ERROR("Didn't have 'data_bit' in config");
            return;
          }
        }
        else
        {
          ROS_ERROR("Didn't have 'parity' in config");
          return;
        }
      }
      else
      {
        ROS_ERROR("Didn't have 'baud_rate' in config");
        return;
      }
      
    }
    else
    {
      ROS_ERROR("Didn't have 'port' in config");
      return;
    }
  }

  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    hardware_interface::JointStateHandle joint_state_handle(iter->first, iter->second->GetPresentPositionPtr(), iter->second->GetPresentVelocityPtr(), iter->second->GetPresentCurrentPtr());
    m_jsi.registerHandle(joint_state_handle);
    ROS_INFO_STREAM("Create JointStateHandle, Name: " + joint_state_handle.getName());
    hardware_interface::JointHandle joint_handle(joint_state_handle, iter->second->GetGoalPositionPtr());
    ROS_INFO_STREAM("Create Actuator Handle, Name: " + joint_handle.getName());
    m_pji.registerHandle(joint_handle);
  }

  registerInterface(&m_jsi);
  registerInterface(&m_pji);

  std::string urdf_string;
  nh.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok())
  {
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh.getParam("robot_description", urdf_string);
    ros::Duration(0.1).sleep();
  }

  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> infos;
  if (!parser.parse(urdf_string, infos))
  {
    ROS_ERROR("Error paring URDF");
    return;
  }

  m_ServiceServer_Autohome = m_nh.advertiseService(ros::this_node::getNamespace() + "Auto_home", &COrientalHardware::AutohomeCallback, this);
  m_ServiceServer_SetProfileVelocity = m_nh.advertiseService(ros::this_node::getNamespace() + "SetProfileVelocity", &COrientalHardware::SetProfileVelocityCallback, this);

}

bool COrientalHardware::AutohomeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    if (!iter->second->auto_home())
    {
      res.success = false;
      res.message = iter->first + " failed auto home";
      return false;
    }
  }

  res.success = true;
  res.message = "All Auto home";
  return true;
}

bool COrientalHardware::SetProfileVelocityCallback(orientalmotor_hardware_msgs::SetProfileVelocityService::Request & req, orientalmotor_hardware_msgs::SetProfileVelocityService::Response &res)
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter)
  {
    double value = req.velocity;
    if (!iter->second->SetProfileVelocity(value))
    {
      res.result = false;
      return false;
    }
  }
  
  res.result = true;
  return true;
}

bool COrientalHardware::init()
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    if (!iter->second->isValid())
    {
      ROS_WARN_STREAM("Actuator: " + iter->first + " wasn't valid");
      return false;
    }
  }

  return true;
}

void COrientalHardware::read()
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    iter->second->read();
  }
}

void COrientalHardware::write()
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    iter->second->write();
  }
}
