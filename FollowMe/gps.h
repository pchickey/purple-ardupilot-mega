
#ifndef __FOLLOWME_GPS_H__
#define __FOLLOWME_GPS_H__

#include <AP_GPS.h>
#include <AP_HAL.h>

bool gps_init(AP_HAL::UARTDriver* uart, GPS* gps);

#endif // __FOLLOWME_GPS_H__

