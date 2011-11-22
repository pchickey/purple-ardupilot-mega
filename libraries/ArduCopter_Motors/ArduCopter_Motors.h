
#ifndef __ARDUCOPTER_MOTORS_H__
#define __ARDUCOPTER_MOTORS_H__

/* The MINIMUM_THROTTLE used to come from ArduCopter/config.h
 * while these files were present in the ArduCopter sketch. Now
 * that they've moved, we need to give the define a new name
 * (to prevent collisions) and define it in this header.
 *
 * If this value needs to be changed frequently by the user
 * from the sketch config, we'll add it to the constructor
 * and make it available as an ivar. It doesn't look like it
 * is changed very often, though.
 */
#define AC_MOTORS_MINIMUM_THROTTLE 130

class ArduCopter_Motors
{
  public:
  virtual void init_out() = 0;
  virtual void output_armed() = 0;
  virtual void output_disarmed() = 0;
  virtual void output_test() = 0;
};

#include "ArduCopter_Motors_Heli.h"
#include "ArduCopter_Motors_Hexa.h"
#include "ArduCopter_Motors_Octa.h"
#include "ArduCopter_Motors_Octa_Quad.h"
#include "ArduCopter_Motors_Quad.h"
#include "ArduCopter_Motors_Tri.h"
#include "ArduCopter_Motors_Y6.h"

#include "ArduCopter_Motors_MulticopterDelegate.h"
#endif // __ARDUCOPTER_MOTORS_H__
