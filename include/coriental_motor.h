#include <string>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>

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

class COrientalMotor
{
private:
  modbus_t* m_ctx;
  uint8_t   m_ID;

  double m_goal_position;
  double m_goal_velcotiy;
  
  double m_present_position;
  double m_present_velocity;

  double m_start_acceleration;
  double m_stop_acceleration;

  OperateMode m_mode;

  bool m_valid;
  
public:
  COrientalMotor(std::string, uint16_t, char, uint8_t, uint8_t, uint8_t id);
  //int set_goal_velocity()

  int move_absolute_position();
  int move_relative_position();

  void write();
  void read();

  double* GetGoalPositionPtr();
  double* GetGoalVelocityPtr();
  double* GetPresentPositionPtr();
  double* GetPresentVelocityPtr();
  
};