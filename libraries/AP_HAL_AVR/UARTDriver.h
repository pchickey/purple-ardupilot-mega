
#ifndef __AP_HAL_AVR_UART_DRIVER_H__
#define __AP_HAL_AVR_UART_DRIVER_H__

#include <stdint.h>

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

/**
 * AVRUARTDriver is an implementation of UARTDriver for the AVR.
 * It will be a thin wrapper on FastSerial.
 */

class AP_HAL_AVR::AVRUARTDriver : public AP_HAL::UARTDriver {
public:
    AVRUARTDriver(int port) {}
    /* Implementations of UARTDriver virtual methods */
    void begin(long b) {}
    void begin(long b, unsigned int rxS, unsigned int txS) {}
    void end() {}
    void flush() {}

    /* Implementations of BetterStream virtual methods */
    int txspace() { return 1; }

    /* Implementations of Stream virtual methods */
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c) { return 0; }
};

#endif // __AP_HAL_AVR_UART_DRIVER_H__

