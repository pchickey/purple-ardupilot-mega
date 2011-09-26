
#include "AP_TimerProcess.h"

extern "C" {
#include <inttypes.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "WConstants.h"
}

int AP_TimerProcess::_period;
void (*AP_TimerProcess::_proc)(void);

AP_TimerProcess::AP_TimerProcess(int period)
{
    _period = period;
    _proc = NULL;
}

void AP_TimerProcess::init(void)
{
	// Enable Timer2 Overflow interrupt to trigger process.
	TIMSK2 = 0;                 // Disable interrupts
	TCCR2A = 0;                 // normal counting mode
	TCCR2B = _BV(CS21) | _BV(CS22);	// Set prescaler of clk/256
	TCNT2  = 0;                 // Set count to zero, so it goes off right away.
	TIFR2  = _BV(TOV2);	        // clear pending interrupts;
	TIMSK2 = _BV(TOIE2);        // enable the overflow interrupt
}

void AP_TimerProcess::register_process(void (*proc)(void) )
{
    _proc = proc;
    TCNT2 = 1;                 // Should go off almost immediately.
}

void AP_TimerProcess::run(void)
{
    TCNT2 = _period;
    if (_proc != NULL)
        _proc();
}

