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
    hardware_interface::JointStateHandle state_handle(iter->first, iter->second->GetPresentPositionPtr(), iter->second->GetPresentVelocityPtr(), iter->second->GetPresentCurrentPtr());
    m_jsi.registerHandle(state_handle);
    ROS_INFO_STREAM("Create JointStateHandle, Name: " + state_handle.getName());
    hardware_interface::JointHandle joint_handle(state_handle, iter->second->GetGoalPositionPtr());
    ROS_INFO_STREAM("Create JointStateHandle, Name: " + joint_handle.getName());
    m_jpi.registerHandle(joint_handle);
  }

  registerInterface(&m_jsi);
  registerInterface(&m_jpi);

  
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
