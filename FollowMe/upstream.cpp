
#include <AP_HAL.h>

#include "upstream.h"

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t upstream_channel;

void upstream_handler(mavlink_channel_t from, mavlink_message_t* msg) {
    hal.console->printf_P(PSTR("Upstream Message %d\r\n"), msg->msgid);
    switch (msg->msgid) {
      default:
        _mavlink_resend_uart(upstream_channel, msg);
    }
}

