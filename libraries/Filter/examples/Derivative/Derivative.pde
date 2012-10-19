/*
 *       Example sketch to demonstrate use of DerivativeFilter library.
 */

#include <stdlib.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <Filter.h>
#include <DerivativeFilter.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

#ifdef DESKTOP_BUILD
#error Desktop build not supported with AP_HAL
#endif


DerivativeFilter<float,11> derivative;

// setup routine
void setup(){}

static float noise(void)
{
    return ((random() % 100)-50) * 0.001;
}

//Main loop where the action takes place
void loop()
{
    hal.scheduler->delay(50);
    float t = hal.scheduler->millis()*1.0e-3;
    float s = sin(t);
    //s += noise();
    uint32_t t1 = hal.scheduler->micros();
    derivative.update(s, t1);
    float output = derivative.slope() * 1.0e6;
    uint32_t t2 = hal.scheduler->micros();
    hal.console->printf("%f %f %f %f\n", t, output, s, cos(t));
}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}

