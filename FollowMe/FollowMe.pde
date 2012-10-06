// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Param.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <GCS_Console.h>

#include <AP_GPS.h>

#include "simplegcs.h"
#include "downstream.h"
#include "upstream.h"
#include "userinput.h"
#include "state.h"

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

mavlink_channel_t upstream_channel = MAVLINK_COMM_1;
mavlink_channel_t downstream_channel = MAVLINK_COMM_0;

GPS* gps;
AP_GPS_Auto auto_gps(hal.uart1, &gps);
FMStateMachine sm;
UserInput input;

static void sm_on_button_activate(int event) {
  if (event == DigitalDebounce::BUTTON_DOWN) {
    sm.on_button_activate();
  }
}

static void sm_on_button_cancel(int event) {
  if (event == DigitalDebounce::BUTTON_DOWN) {
    sm.on_button_cancel();
  }
}

void setup(void) {
    /* Allocate large enough buffers on uart0 to support mavlink */
    hal.uart0->begin(115200, 256, 256);
    hal.uart2->begin(115200, 256, 256);
    /* Don't need such big buffers for GPS */
    hal.uart1->begin(57600, 256, 16);


    /* Setup GCS_Mavlink library's comm 0 port. */
    mavlink_comm_0_port = hal.uart0;
    /* Setup GCS_Mavlink library's comm 1 port to UART2 (accessible on APM2) */
    mavlink_comm_1_port = hal.uart2;
    
    simplegcs_send_heartbeat(downstream_channel);

    hal.scheduler->register_timer_process(simplegcs_send_console_async, 1, 0);
    hal.scheduler->register_timer_process(simplegcs_send_heartbeat_async, 1, 0);
    hal.console->backend_open();
    hal.scheduler->delay(1000);
    hal.console->println_P(PSTR("Hello hal.console"));

    hal.console->println_P(PSTR("User input init"));
    input.init(57, 0, 1, 51);
    input.side_btn_event_callback(sm_on_button_activate);
    input.joy_btn_event_callback(sm_on_button_cancel);

    hal.console->println_P(PSTR("GPS start init"));
    auto_gps.init(GPS::GPS_ENGINE_PEDESTRIAN);
}

void loop(void) {
    sm.on_loop(gps); 
    if (gps != NULL) {
        gps->update();
        if (gps->new_data) {
            if (gps->fix) {
                hal.console->printf_P(
                    PSTR("GPS lat %ld lon %ld alt %.2f\r\n"), 
                    gps->latitude, gps->longitude,
                    (float) gps->altitude / 100.0);
            } else {
                hal.console->println_P(PSTR("GPS nofix"));
            }
            gps->new_data = false;
        }
    } else {
        auto_gps.update();
    }

#if DEBUG_USERINPUT
    input.print(hal.console);
#endif
    /* Receive messages off the downstream, send them upstream: */
    simplegcs_update(downstream_channel, upstream_handler);
    /* Receive messages off the downstream, send them upstream: */
    simplegcs_update(upstream_channel, downstream_handler);

    hal.scheduler->delay(100);
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
