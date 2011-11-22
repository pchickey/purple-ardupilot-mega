
#ifndef __ARDUCOPTER_MOTORS_MULTICOPTER_DELEGATE_H__
#define __ARDUCOPTER_MOTORS_MULTICOPTER_DELEGATE_H__

#include <stdint.h>
#include "../RC_Channel/RC_Channel.h"

/* Pure virtual ArduCopter_Motors_MulticopterDelegate class is an interface
 * to be implemented by the ArduCopter Parameters class. It is used
 * for ArduCopter_Motors constructors to access the rc_n and
 * frame_orientation members of the Parameters class without explicitly
 * knowing about the Parameters class, which we can't do because it is
 * defined in the sketch, not the libraries.
 */

class ArduCopter_Motors_MulticopterDelegate {
  public:
  virtual RC_Channel * get_rc_1() = 0;
  virtual RC_Channel * get_rc_2() = 0;
  virtual RC_Channel * get_rc_3() = 0;
  virtual RC_Channel * get_rc_4() = 0;
  virtual RC_Channel * get_rc_5() = 0;
  virtual RC_Channel * get_rc_6() = 0;
  virtual RC_Channel * get_rc_7() = 0;
  virtual RC_Channel * get_rc_8() = 0;
  virtual int8_t       get_frame_orientation() = 0;
  virtual float        get_top_bottom_ratio() = 0;
};

#endif // __ARDUCOPTER_MOTORS_MULTICOPTER_DELEGATE_H__

