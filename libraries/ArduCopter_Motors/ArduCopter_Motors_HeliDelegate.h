
#ifndef __ARDUCOPTER_MOTORS_HELI_DELEGATE_H__
#define __ARDUCOPTER_MOTORS_HELI_DELEGATE_H__

#include <stdint.h>
#include "../RC_Channel/RC_Channel.h"

/* Pure virtual ArduCopter_Motors_HeliDelegate class is an interface
 * to be implemented by the ArduCopter Parameters class. It is used
 * for the ArduCopter_Motors_Heli constructor to access the members of
 * the Parameters class without explicitly knowing about the Parameters
 * class, which we can't do because it is defined in the sketch, not 
 * the libraries.
 */

class ArduCopter_Motors_HeliDelegate {
  // XXX stub
};

#endif // __ARDUCOPTER_MOTORS_HELI_DELEGATE_H__

