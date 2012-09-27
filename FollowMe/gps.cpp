
#include <AP_HAL.h>
#include "gps.h"

extern const AP_HAL::HAL& hal;

bool gps_init(AP_HAL::UARTDriver* uart, GPS* gps) {
    AP_GPS_Auto auto_gps(uart, &gps);
    auto_gps.init(GPS::GPS_ENGINE_PEDESTRIAN);
    
    int gps_search = 0;
    do { auto_gps.update();
         if (auto_gps.status() != 0) {
            return true;
         }
         gps_search++;
    } while (gps_search < 3);

    hal.console->println_P(PSTR("Error: No GPS Found"));

}
