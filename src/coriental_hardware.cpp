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
          if (motor_config.getParam("data_bit", param.data_bit))
          {
            if (motor_config.getParam("stop_bit", param.stop_bit))
            {
              if (motor_config.getParam("id", param.id))
              {
                if (!motor_config.getParam("max_position", max_position))
                {
                  ROS_ERROR("Didn't have 'max_position' in config");
                  return;
                }


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


}
