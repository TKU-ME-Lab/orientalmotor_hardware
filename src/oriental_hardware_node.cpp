#include "coriental_motor.h"
#include <iostream>


int main(void)
{
  COrientalMotor motor("/dev/U2D_AL01QGU9", 115200, 'E', 8, 1, 1);
  


  return 0;
}