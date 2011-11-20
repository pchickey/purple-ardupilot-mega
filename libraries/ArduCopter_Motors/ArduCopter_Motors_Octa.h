
#ifndef __ARDUCOPTER_MOTORS_OCTA_H__
#define __ARDUCOPTER_MOTORS_OCTA_H__

#include "ArduCopter_Motors.h"

class ArduCopter_Motors_Octa : public ArduCopter_Motors
{
  public:
  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
};

#endif // __ARDUCOPTER_MOTORS_OCTA_H__

