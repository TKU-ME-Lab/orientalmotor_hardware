#include <string>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>

#define OPERATE_CMD(bit) (1 << bit)

const uint16_t OPERATE_CMD_M0         =  0;
const uint16_t OPERATE_CMD_M1         =  1;
const uint16_t OPERATE_CMD_M2         =  2;
const uint16_t OPERATE_CMD_START      =  3;
const uint16_t OPERATE_CMD_ZHOME      =  4;
const uint16_t OPERATE_CMD_STOP       =  5;
const uint16_t OPERATE_CMD_FREE       =  6;
const uint16_t OPERATE_CMD_ALM_RST    =  7;
const uint16_t OPERATE_CMD_D_SEL0     =  8;
const uint16_t OPERATE_CMD_D_SEL1     =  9;
const uint16_t OPERATE_CMD_D_SEL2     = 10;
const uint16_t OPERATE_CMD_SSTART     = 11;
const uint16_t OPERATE_CMD_FW_JOG_P   = 12;
const uint16_t OPERATE_CMD_RV_JOG_P   = 13;
const uint16_t OPERATE_CMD_FW_POS     = 14;
const uint16_t OPERATE_CMD_RV_POS     = 15;

//Registers Address
const uint16_t OPERATE_CMD_ADDR    = 125;

const uint16_t GOAL_POSITION_UPPER = 198;
const uint16_t GOAL_POSITION_LOWER = 199;
const uint16_t GOAL_VELOCITY_UPPER = 200;
const uint16_t GOAL_VELOCITY_LOWER = 201;

const uint16_t PRESENT_POSITION_UPPER = 204;
const uint16_t PRESENT_POSITION_LOWER = 205;
const uint16_t PRESENT_VELOCITY_UPPER = 206;
const uint16_t PRESENT_VELOCITY_LOWER = 207;

const uint16_t RUNNING_DATA_NO_0_ADDRESS     = 6144;
const uint16_t RUNNING_DATA_ADDRESS_INTERVAL = 64;
const uint16_t RUNNING_DATA_COUNT            = 256;

const uint16_t DEFAULT_CURRENT_UPPER = 588;
const uint16_t DEFAULT_CURRENT_LOWER = 589;

typedef enum
{
  Absolute = 1, Relative
}OperateMode;

typedef struct{
  unsigned short id;
  std::string port_name;
  int  baud_rate;
  char parity;
  unsigned short data_bit;
  unsigned short stop_bit;
  float profile_velocity;
}OrientalParammeter;

class COrientalActuator
{
private:
  modbus_t* m_ctx;
  uint8_t   m_ID;

  float m_goal_position;
  float m_goal_velcotiy;
  
  float m_present_position;
  float m_present_velocity;

  bool m_valid;
  
public:
  COrientalActuator(OrientalParammeter);
  ~COrientalActuator();

  void write();
  void read();

  float* GetGoalPositionPtr();
  float* GetGoalVelocityPtr();
  float* GetPresentPositionPtr();
  float* GetPresentVelocityPtr();
  
};