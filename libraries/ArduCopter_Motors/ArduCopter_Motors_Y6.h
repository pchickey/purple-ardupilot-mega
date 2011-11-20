
#ifndef __ARDUCOPTER_MOTORS_Y6_H__
#define __ARDUCOPTER_MOTORS_Y6_H__

#include "ArduCopter_Motors.h"

class ArduCopter_Motors_Y6 : public ArduCopter_Motors
{
  public:
  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
};

#endif // __ARDUCOPTER_MOTORS_Y6_H__

