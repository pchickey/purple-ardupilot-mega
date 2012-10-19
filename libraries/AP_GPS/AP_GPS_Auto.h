// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Auto-detecting pseudo-GPS driver
//

#ifndef __AP_GPS_AUTO_H__
#define __AP_GPS_AUTO_H__

#include <AP_HAL.h>
#include "GPS.h"

class AP_GPS_Auto : public GPS
{
public:
    /// Constructor
    ///
    /// @note The stream is expected to be set up and configured for the
    ///       correct bitrate before ::init is called.
    ///
    /// @param	port	UARTDriver connected to the GPS module.
    /// @param	ptr		Pointer to a GPS * that will be fixed up by ::init
    ///					when the GPS type has been detected.
    ///
    AP_GPS_Auto(AP_HAL::UARTDriver *s, GPS **gps);

    /// Dummy init routine, does nothing
    virtual void        init(enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);

    /// Detect and initialise the attached GPS unit.  Updates the
    /// pointer passed into the constructor when a GPS is detected.
    ///
    virtual bool        read(void);

private:
    /// global GPS driver pointer, updated by auto-detection
    ///
    GPS **     _gps;

    /// low-level auto-detect routine
    ///
    GPS *                           _detect(void);

    static const prog_char          _mtk_set_binary[];
    static const prog_char          _sirf_set_binary[];

    enum GPS_Engine_Setting         _nav_setting;
};
#endif
