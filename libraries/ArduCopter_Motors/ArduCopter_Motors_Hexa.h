
#ifndef __ARDUCOPTER_MOTORS_HEXA_H__
#define __ARDUCOPTER_MOTORS_HEXA_H__

#include <stdint.h>
#include "../APM_RC/APM_RC.h"
#include "ArduCopter_Motors.h"

class ArduCopter_Motors_Hexa : public ArduCopter_Motors
{
  public:
  ArduCopter_Motors_Hexa(APM_RC_Class *apm_rc, int16_t *motor_out) :
    _apm_rc(apm_rc) , _motor_out(motor_out) {}

  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
  private:
  APM_RC_Class *_apm_rc;
  int16_t *_motor_out;
};

#endif // __ARDUCOPTER_MOTORS_HEXA_H__

