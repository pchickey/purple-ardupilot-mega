

#include <AP_HAL.h>

#include "downstream.h"

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t downstream_channel;

void downstream_handler(mavlink_channel_t from, mavlink_message_t* msg) {
    switch (msg->msgid) {
      default:
        _mavlink_resend_uart(downstream_channel, msg);
    }
}

