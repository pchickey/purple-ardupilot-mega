
#ifndef __ARDUCOPTER_MOTORS_Y6_H__
#define __ARDUCOPTER_MOTORS_Y6_H__

#include "ArduCopter_Motors.h"

class ArduCopter_Motors_Y6 : public ArduCopter_Motors
{
  public:
  ArduCopter_Motors_Y6(APM_RC_Class *apm_rc) :
    _apm_rc(apm_rc) {}

  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
  private:
  APM_RC_Class *_apm_rc;
};

#endif // __ARDUCOPTER_MOTORS_Y6_H__

