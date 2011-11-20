
#ifndef __ARDUCOPTER_MOTORS_H__
#define __ARDUCOPTER_MOTORS_H__

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

#endif // __ARDUCOPTER_MOTORS_H__

