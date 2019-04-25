#include "coriental_hardware.h"
#include <boost/foreach.hpp>

COrientalHardware::COrientalHardware(ros::NodeHandle& nh, ros::NodeHandle& private_nh, std::vector<std::string> motor_names):
  m_nh(nh), m_private_nh(private_nh)
{
  BOOST_FOREACH(const std::string motor_name, motor_names)
  {
    try{
      m_transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &m_robot_transmissions));
    }
    catch (const std::invalid_argument& ex){
      ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
      return;
    }
    catch(...){
      ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
      return;
    }

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
    hardware_interface::ActuatorStateHandle state_handle(iter->first, iter->second->GetPresentPositionPtr(), iter->second->GetPresentVelocityPtr(), iter->second->GetPresentCurrentPtr());
    m_asi.registerHandle(state_handle);
    ROS_INFO_STREAM("Create JointStateHandle, Name: " + state_handle.getName());
    hardware_interface::ActuatorHandle actuator_handle(state_handle, iter->second->GetGoalPositionPtr());
    ROS_INFO_STREAM("Create Actuator Handle, Name: " + actuator_handle.getName());
    m_api.registerHandle(actuator_handle);
  }

  registerInterface(&m_asi);
  registerInterface(&m_api);

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

  std::vector<std::string> actuator_names;
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    actuator_names.push_back(iter->first);
  }

  BOOST_FOREACH(const transmission_interface::TransmissionInfo& info, infos){
    bool found_some = false;
    bool found_all = true;
    BOOST_FOREACH(const transmission_interface::ActuatorInfo& actuator, info.actuators_)
    {
      if (std::find(actuator_names.begin(), actuator_names.end(), actuator.name_) != actuator_names.end())
      {
        found_some = true;
      }
      else{
        found_all = false;
      }
    }

    if (found_all)
    {
      if (!m_transmission_loader->load(info))
      {
        ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
        return;
      }
      else
      {
        ROS_INFO_STREAM("Loaded transmission: " << info.name_);
      }
    }
    else if(found_some)
    {
      ROS_ERROR_STREAM("Don't support transmission that contain only some Oriental Actuator: " << info.name_);
    }
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
  if (m_robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>()){
    m_robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
  }
}

void COrientalHardware::write()
{
  for (OrientalMotorMap::iterator iter = m_OrientalMotorMap.begin(); iter != m_OrientalMotorMap.end(); iter++)
  {
    iter->second->write();
  }
  if (m_robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>()){
    m_robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
  }
}
