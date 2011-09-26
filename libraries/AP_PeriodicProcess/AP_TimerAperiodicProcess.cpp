
#include "AP_TimerAperiodicProcess.h"

extern "C" {
#include <inttypes.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "WConstants.h"
}
// TCNT2 values for various interrupt rates,
// assuming 256 prescaler. Note that these values
// assume a zero-time ISR. The actual rate will be a
// bit lower than this
#define TCNT2_781_HZ   (256-80)
#define TCNT2_1008_HZ  (256-62)
#define TCNT2_1302_HZ  (256-48)

uint8_t AP_TimerAperiodicProcess::_timer_offset;

void AP_TimerAperiodicProcess::run(void)
{
    _timer_offset = (_timer_offset + 49) % 32;
    _period = TCNT2_781_HZ + _timer_offset;
    TCNT2 = _period;
    if (_proc != NULL)
        _proc();
}

