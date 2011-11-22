
#ifndef __ARDUCOPTER_MOTORS_HELI_H__
#define __ARDUCOPTER_MOTORS_HELI_H__

#include <stdint.h>
#include "../APM_RC/APM_RC.h"
#include "ArduCopter_Motors.h"
#include "ArduCopter_Motors_HeliDelegate.h"

class ArduCopter_Motors_Heli : public ArduCopter_Motors
{
  public:
  ArduCopter_Motors_Heli(ArduCopter_Motors_HeliDelegate *g,
                       APM_RC_Class *apm_rc, int16_t *motor_out, boolean *auto_armed) :
    _g(g), _apm_rc(apm_rc), _motor_out(motor_out), _auto_armed(auto_armed) {}

  void init();
  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
  private:
  void _init_g();

  ArduCopter_Motors_HeliDelegate *_g;
  APM_RC_Class *_apm_rc;
  int16_t *_motor_out;
  boolean *_auto_armed;
};

#endif // __ARDUCOPTER_MOTORS_HELI_H__

