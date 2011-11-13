// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	This has the basic functions that all RangeFinders need implemented
*/

// AVR LibC Includes
#include "WConstants.h"
#include "RangeFinder.h"



// Public Methods //////////////////////////////////////////////////////////////

void RangeFinder::set_orientation(int x, int y, int z)
{
    orientation_x = x;
	orientation_y = y;
	orientation_z = z;
}

// Read Sensor data - only the raw_value is filled in by this parent class
int RangeFinder::read()
{
	raw_value = _analog_source->read();

	raw_value = convert_raw_to_distance(raw_value);

	// convert analog value to distance in cm (using child implementation most likely)
	raw_value = convert_raw_to_distance(raw_value);

	// ensure distance is within min and max
	raw_value = constrain(raw_value, min_distance, max_distance);

	distance = _mode_filter->get_filtered_with_sample(raw_value);
	return distance;
}

