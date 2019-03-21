#include "coriental_motor.h"
#include <iostream>

COrientalMotor::COrientalMotor(std::string device_name, unsigned int baudrate, char parity, unsigned short data_bit, unsigned short stop_bit, unsigned short id):
                              m_ID(id)
{
  m_ctx = modbus_new_rtu(device_name.c_str(), baudrate, parity, data_bit, stop_bit);
  if (m_ctx == NULL)
  {
    std::cout << "Failed to create modbus context" << std::endl;
    std::cout << "Device: " << device_name << ", Baudrate:" << baudrate << ", Parity:" << parity 
              << ", Data Bit Size: " << data_bit << ", Stop Bit Size: " << stop_bit << std::endl; 
    return;
  }

  modbus_set_slave(m_ctx, m_ID);

  if (modbus_connect(m_ctx) == -1)
  {
    std::cout << "Modbus failed to connect device" << std::endl;
    modbus_free(m_ctx);
    return;
  }

  m_valid = true;
}

int COrientalMotor::move_absolute_position()
{
  

  return 0;
}

int COrientalMotor::move_relative_position()
{

}

void COrientalMotor::write()
{
  return;
}

void COrientalMotor::read()
{
  uint16_t buffer[2];

  int result = 0;

  result = modbus_read_registers(m_ctx, PRESENT_POSITION_UPPER, 2, buffer);
  if (result == -1)
  {
    std::cout << "[OrientalMotor] ID: " << m_ID << ", failed to read position registers " << std::endl;
    return;
  }

  m_present_position = (double) ((buffer[0] << 16) | buffer[1]) /100;

  #ifdef _DEBUG
    std::cout << "Position: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
    std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;
  #endif

  modbus_read_registers(m_ctx, PRESENT_VELOCITY_UPPER, 2, buffer);
  m_present_velocity = (double) ((buffer[0] << 16) | buffer[1]) /100;

  #ifdef _DEBUG
    std::cout << "Velocity: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
    std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;
  #endif
}

double* COrientalMotor::GetGoalPositionPtr()
{
  return &m_goal_position;  
}

double* COrientalMotor::GetGoalVelocityPtr()
{
  return &m_goal_velcotiy;
}

double* COrientalMotor::GetPresentPositionPtr()
{
  return &m_present_position;
}

double* COrientalMotor::GetPresentVelocityPtr()
{
  return &m_present_velocity;
}
