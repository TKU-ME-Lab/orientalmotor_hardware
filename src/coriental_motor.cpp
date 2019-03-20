#include "coriental_motor.h"
#include <iostream>

COrientalMotor::COrientalMotor(std::string device_name, uint16_t baudrate, char parity, uint8_t data_bit, uint8_t stop_bit, uint8_t id):
                              m_ID(id)
{
  m_ctx = modbus_new_rtu(device_name.c_str(), baudrate, parity, data_bit, stop_bit);
  if (m_ctx == NULL)
  {
    std::cout << "Failed to create modbus context" << std::endl;
    return;
  }

  modbus_set_slave(m_ctx, m_ID);

  if (modbus_connect(m_ctx) == -1)
  {
    std::cout << "Modbus failed to connect device" << std::endl;
    modbus_free(m_ctx);
    return;
  }
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

  modbus_read_registers(m_ctx, PRESENT_POSITION_UPPER, 2, buffer);
  std::cout << "Position: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
  std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;

  modbus_read_registers(m_ctx, PRESENT_VELOCITY_UPPER, 2, buffer);
  std::cout << "Velocity: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
  std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;

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
