#include "coriental_actuator.h"
#include <iostream>

COrientalActuator::COrientalActuator(OrientalParammeter param):
                              m_ID(param.id), m_goal_velcotiy(param.profile_velocity), m_valid(false), m_inited_home(false)
{
  m_ctx = modbus_new_rtu(param.port_name.c_str(), param.baud_rate, param.parity, param.data_bit, param.stop_bit);
  if (m_ctx == NULL)
  {
    std::cout << "Failed to create modbus context" << std::endl;
    std::cout << "Device: " << param.port_name << ", Baudrate:" << param.baud_rate << ", Parity:" << param.parity 
              << ", Data Bit Size: " << param.data_bit << ", Stop Bit Size: " << param.stop_bit << std::endl; 
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

COrientalActuator::~COrientalActuator()
{
  modbus_free(m_ctx);
}

void COrientalActuator::write()
{ 
  if (!m_valid)
  {
    return;
  }

  if (m_inited_home){
    if (m_operate_status & OPERATE(OUTPUT_STATUS_MOVE))
    {
      //std::cout << "Is Moving!" << std::endl;
      return;
    }

    int result = 0;

    uint32_t data_int = (m_goal_position * 200000);
    uint16_t buffer[2];  
    buffer[0] = data_int >> 16;
    buffer[1] = data_int & 0xffff;

    result = modbus_write_register(m_ctx, OPERATE_CMD_ADDR, OPERATE(INPUT_CMD_STOP));

    result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS  , 0);
    result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS+1, 1); 
    result = modbus_write_registers(m_ctx, RUNNING_DATA_NO_0_ADDRESS+2, 2, buffer);

    // data_int = (m_goal_velcotiy * 200);
    // buffer[0] = data_intisValid >> 16;
    // buffer[1] = data_int & 0xffff;
    // result = modbus_write_registers(m_ctx, RUNNING_DATA_NO_0_ADDRESS+4, 2, buffer);

    result = modbus_write_register(m_ctx, OPERATE_CMD_ADDR, OPERATE(INPUT_CMD_START));
  }
  
  return;
}

void COrientalActuator::read()
{
  if (!m_valid)
  {
    return;
  }

  uint16_t buffer[2];

  int result = 0;

  result = modbus_read_registers(m_ctx, PRESENT_POSITION_UPPER, 2, buffer);
  if (result == -1)
  {
    std::cout << "[OrientalMotor] ID: " << m_ID << ", failed to read position registers " << std::endl;
    return;
  }

  m_present_position = (double) ((buffer[0] << 16) | buffer[1]) /200000; //[m]

  #ifdef _DEBUG
    // std::cout << "Position: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
    // std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;
    std::cout << "Position: " << m_present_position <<  "  (m)" << std::endl;
  #endif

  modbus_read_registers(m_ctx, PRESENT_VELOCITY_UPPER, 2, buffer);
  m_present_velocity = (double) ((buffer[0] << 16) | buffer[1]) / 12000; //[m/s]

  #ifdef _DEBUG
    // std::cout << "Velocity: " << buffer[0] << " [Upper], " << buffer[1] << " [Lower] (DEC)" << std::endl;
    // std::cout << "          " << std::hex << buffer[0] << " [Upper], " << std::hex << buffer[1] << " [Lower] (HEX)" << std::endl;
    std::cout << "          " << m_present_velocity <<  "  (m/s)" << std::endl;
  #endif

  modbus_read_registers(m_ctx, OPERATE_STATUS_ADDR, 1, &m_operate_status);
}

bool COrientalActuator::isValid()
{
  return m_valid;
}

bool COrientalActuator::auto_home()
{
  int result = 0;
  uint16_t buffer[2];  
  buffer[0] = 0;
  buffer[1] = 0;
    
  result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS  , 0);
  result = modbus_write_register( m_ctx, RUNNING_DATA_NO_0_ADDRESS+1, 1); 
  result = modbus_write_registers(m_ctx, RUNNING_DATA_NO_0_ADDRESS+2, 2, buffer);

  // data_int = (m_goal_velcotiy * 200);
  // buffer[0] = data_int >> 16;
  // buffer[1] = data_int & 0xffff;
  // result = modbus_write_registers(m_ctx, RUNNING_DATA_NO_0_ADDRESS+4, 2, buffer);

  result = modbus_write_register(m_ctx, OPERATE_CMD_ADDR, OPERATE(INPUT_CMD_START));
  if (result)
  {
    m_inited_home = true;
  }

  return result;
}

double* COrientalActuator::GetGoalPositionPtr()
{
  return &m_goal_position;  
}

double* COrientalActuator::GetGoalVelocityPtr()
{
  return &m_goal_velcotiy;
}

double* COrientalActuator::GetPresentPositionPtr()
{
  return &m_present_position;
}

double* COrientalActuator::GetPresentVelocityPtr()
{
  return &m_present_velocity;
}


double* COrientalActuator::GetPresentCurrentPtr()
{
  return &m_present_current;
}