
#ifndef __ARDUCOPTER_MOTORS_OCTA_H__
#define __ARDUCOPTER_MOTORS_OCTA_H__

#include "ArduCopter_Motors.h"
#include "ArduCopter_Motors_Multicopter.h"

class ArduCopter_Motors_Octa :
    public ArduCopter_Motors
  , public ArduCopter_Motors_Multicopter
{
  public:
  ArduCopter_Motors_Octa(ArduCopter_Motors_MulticopterDelegate *g,
                         APM_RC_Class *apm_rc ,
                         int16_t *motor_out   ,
                         boolean *auto_armed  ) :
    ArduCopter_Motors_Multicopter(g, apm_rc, motor_out, auto_armed) {}

  void init();
  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
};

#endif // __ARDUCOPTER_MOTORS_OCTA_H__
