
#ifndef __ARDUCOPTER_MOTORS_MULTICOPTER_H__
#define __ARDUCOPTER_MOTORS_MULTICOPTER_H__

#include <stdint.h>

#include <APM_RC.h>
#include <RC_Channel.h>

#include "ArduCopter_Motors_MulticopterDelegate.h"

class ArduCopter_Motors_Multicopter
{
  public:
  ArduCopter_Motors_Multicopter(ArduCopter_Motors_MulticopterDelegate *g,
                                APM_RC_Class *apm_rc ,
                                int16_t *motor_out   ,
                                boolean *auto_armed  ) :
    _g(g), _apm_rc(apm_rc), _motor_out(motor_out), _motor_auto_armed(auto_armed) {}
  protected:
  void _init_g();
  ArduCopter_Motors_MulticopterDelegate *_g;
  RC_Channel   *_rc_1;
  RC_Channel   *_rc_2;
  RC_Channel   *_rc_3;
  RC_Channel   *_rc_4;
  RC_Channel   *_rc_5;
  RC_Channel   *_rc_6;
  RC_Channel   *_rc_7;
  RC_Channel   *_rc_8;
  int8_t        _frame_orientation;
  float         _top_bottom_ratio;

  int16_t      *_motor_out;
  APM_RC_Class *_apm_rc;
  boolean      *_motor_auto_armed;
};


#endif // __ARDUCOPTER_MOTORS_MULTICOPTER_H__
