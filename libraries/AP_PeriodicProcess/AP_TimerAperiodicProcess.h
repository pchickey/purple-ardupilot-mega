
#ifndef __AP_TIMER_APERIODIC_PROCESS_H__
#define __AP_TIMER_APERIODIC_PROCESS_H__

#include <stdint.h>

#include "AP_TimerProcess.h"

class AP_TimerAperiodicProcess : public AP_TimerProcess
{
    public:
        static void run(void);
    private:
        static uint8_t _timer_offset;
};

#endif // __AP_TIMER_APERIODIC_PROCESS_H__

