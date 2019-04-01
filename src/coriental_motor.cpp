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

COrientalMotor::~COrientalMotor()
{
  
}

int COrientalMotor::move_absolute_position(float data)
{
  int result = 0;
  result = modbus_write_register(m_ctx, OPERATE_CMD_ADDR, 0);

  uint32_t data_int = (data * 100);
  uint16_t buffer[2];
  buffer[0] = data_int >> 16;
  buffer[1] = data_int & 0xffff;
  
  result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS  , 0);
  result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS+1, 1); 
  result = modbus_write_registers(m_ctx, RUNNING_DATA_NO_0_ADDRESS+2, 2, buffer);


  result = modbus_write_register(m_ctx, OPERATE_CMD_ADDR, OPERATE_CMD(OPERATE_CMD_START));

  return 0;
}

int COrientalMotor::move_relative_position()
{

}

void COrientalMotor::write()
{
  if (m_mode == OperateMode::Absolute)
  {
    
  }

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

float* COrientalMotor::GetGoalPositionPtr()
{
  return &m_goal_position;  
}

float* COrientalMotor::GetGoalVelocityPtr()
{
  return &m_goal_velcotiy;
}

float* COrientalMotor::GetPresentPositionPtr()
{
  return &m_present_position;
}

float* COrientalMotor::GetPresentVelocityPtr()
{
  return &m_present_velocity;
}
