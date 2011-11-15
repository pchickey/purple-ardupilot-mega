#ifndef __APM_RC_H__
#define __APM_RC_H__

#include <inttypes.h>

// Radio channels
// Note channels are from 0!
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_10 9
#define CH_11 10

#define NUM_CHANNELS 8

class APM_RC_Class
{
  public:
	APM_RC_Class() {}
	virtual void OutputCh(uint8_t ch, uint16_t pwm) = 0;
	virtual uint16_t InputCh(uint8_t ch) = 0;
	virtual uint8_t GetState() = 0;
	virtual void clearOverride(void) = 0;
    virtual void Force_Out() = 0;
};

#include "APM_RC_APM1.h"
#include "APM_RC_Purple.h"

#endif
