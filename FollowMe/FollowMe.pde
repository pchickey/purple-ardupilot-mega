// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Param.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <GCS_Console.h>

#include <AP_GPS.h>

#include "gps.h"
#include "simplegcs.h"
#include "downstream.h"
#include "upstream.h"
#include "userinput.h"

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

mavlink_channel_t upstream_channel = MAVLINK_COMM_1;
mavlink_channel_t downstream_channel = MAVLINK_COMM_0;

GPS* gps;

void console_loopback() {
    int a = hal.console->available();
    if (a > 0) {
        hal.console->print("Console loopback:");
        int r = hal.console->read();
        while (r > 0) {
            hal.console->write( (uint8_t) r );
            r = hal.console->read();
        }
        hal.console->println();
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

    UserInput::init(54, 4, 1, 0);

    bool gps_found = gps_init(hal.uart1, gps);

    /* Debug - make sure i setup the gps auto stuff correctly*/
    if (gps_found && gps == NULL) {
      hal.console->println_P(PSTR("Fail: No gps object - serious error"));
    }

}

void loop(void) {
    if (gps != NULL) {
      hal.console->printf_P(PSTR("FM GPS lat %ld lon %ld\r\n"), 
            gps->latitude, gps->longitude);
    }
//    UserInput::print(hal.console);

    /* Receive messages off the downstream, send them upstream: */
    simplegcs_update(downstream_channel, upstream_handler);
    /* Receive messages off the downstream, send them upstream: */
    simplegcs_update(upstream_channel, downstream_handler);

    console_loopback();
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
