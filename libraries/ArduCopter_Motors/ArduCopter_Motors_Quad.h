
#ifndef __ARDUCOPTER_MOTORS_QUAD_H__
#define __ARDUCOPTER_MOTORS_QUAD_H__

#include "../APM_RC/APM_RC.h"
#include "ArduCopter_Motors.h"

class ArduCopter_Motors_Quad : public ArduCopter_Motors
{
  public:
  ArduCopter_Motors_Quad(APM_RC_Class *apm_rc) :
    _apm_rc(apm_rc) {}

  void init_out();
  void output_armed();
  void output_disarmed();
  void output_test();
  private:
  APM_RC_Class *_apm_rc;
};

#endif // __ARDUCOPTER_MOTORS_QUAD_H__

