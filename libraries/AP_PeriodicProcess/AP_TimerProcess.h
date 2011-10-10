
#ifndef __AP_TIMERPROCESS_H__
#define __AP_TIMERPROCESS_H__

#include "PeriodicProcess.h"

/* XXX this value is a total guess, will look up. */
#define TIMERPROCESS_PER_DEFAULT (256)

class AP_TimerProcess : public AP_PeriodicProcess
{
    public:
        AP_TimerProcess(int period = TIMERPROCESS_PER_DEFAULT);
        void init();
        void register_process(void (* proc)(void));
        static void run(void);
    protected:
        static int  _period; 
        static void (*_proc)(void);
};

#endif // __AP_TIMERPROCESS_H__

