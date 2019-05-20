#include <string>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>

#define OPERATE(bit) (1 << bit)

//
const uint16_t INPUT_CMD_M0         =  0;
const uint16_t INPUT_CMD_M1         =  1;
const uint16_t INPUT_CMD_M2         =  2;
const uint16_t INPUT_CMD_START      =  3;
const uint16_t INPUT_CMD_ZHOME      =  4;
const uint16_t INPUT_CMD_STOP       =  5;
const uint16_t INPUT_CMD_FREE       =  6;
const uint16_t INPUT_CMD_ALM_RST    =  7;
const uint16_t INPUT_CMD_D_SEL0     =  8;
const uint16_t INPUT_CMD_D_SEL1     =  9;
const uint16_t INPUT_CMD_D_SEL2     = 10;
const uint16_t INPUT_CMD_SSTART     = 11;
const uint16_t INPUT_CMD_FW_JOG_P   = 12;
const uint16_t INPUT_CMD_RV_JOG_P   = 13;
const uint16_t INPUT_CMD_FW_POS     = 14;
const uint16_t INPUT_CMD_RV_POS     = 15;

const uint16_t OUTPUT_STATUS_M0_R     =  0;
const uint16_t OUTPUT_STATUS_M1_R     =  1;
const uint16_t OUTPUT_STATUS_M2_R     =  2;
const uint16_t OUTPUT_STATUS_START    =  3;
const uint16_t OUTPUT_STATUS_HOME_END =  4;
const uint16_t OUTPUT_STATUS_READY    =  5;
const uint16_t OUTPUT_STATUS_INFO     =  6;
const uint16_t OUTPUT_STATUS_ALM_A    =  7;
const uint16_t OUTPUT_STATUS_SYS_BSY  =  8;
const uint16_t OUTPUT_STATUS_AREA0    =  9;
const uint16_t OUTPUT_STATUS_AREA1    = 10;
const uint16_t OUTPUT_STATUS_AREA2    = 11;
const uint16_t OUTPUT_STATUS_TIM      = 12;
const uint16_t OUTPUT_STATUS_MOVE     = 13;
const uint16_t OUTPUT_STATUS_IN_POS   = 14;
const uint16_t OUTPUT_STATUS_TLC      = 15;

//Registers Address
const uint16_t OPERATE_CMD_ADDR    = 125;
const uint16_t OPERATE_STATUS_ADDR = 127;

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
//Registers Address

typedef enum
{
  Absolute = 1, Relative
}OperateMode;

typedef struct{
  unsigned short id;
  std::string port_name;
  std::string joint_name;
  int  baud_rate;
  char parity;
  unsigned short data_bit;
  unsigned short stop_bit;
  double profile_velocity;
}OrientalParammeter;

class COrientalActuator
{
private:
  modbus_t* m_ctx;
  uint8_t   m_ID;

  uint16_t m_operate_status;

  double m_goal_position;
  double m_goal_velcotiy;
  
  double m_present_position;
  double m_present_velocity;
  double m_present_current;

  bool m_valid;
  bool m_inited_home;

  std::string m_joint_name;
public:
  COrientalActuator(OrientalParammeter);
  ~COrientalActuator();

  void write();
  void read();

  bool isValid();

  bool auto_home();
  bool SetProfileVelocity(double);

  double* GetGoalPositionPtr();
  double* GetGoalVelocityPtr();
  double* GetPresentPositionPtr();
  double* GetPresentVelocityPtr();
  double* GetPresentCurrentPtr();
  
  std::string GetJointName(){return m_joint_name;}
};