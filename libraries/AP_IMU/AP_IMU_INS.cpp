// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//
//	AP_IMU_INS.cpp - IMU Sensor Library for Ardupilot Mega
//		Code by Michael Smith, Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

/// @file	AP_IMU_INS.cpp
/// @brief	IMU driver on top of an INS driver. Provides calibration for the
//          inertial sensors (gyro and accel)

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/eeprom.h>

#include "AP_IMU_INS.h"

// XXX secret knowledge about the APM/oilpan wiring
//
#define A_LED_PIN   37
#define C_LED_PIN   35

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
//
const float   AP_IMU_INS::_gyro_temp_curve[3][3] = {
	{1658,0,0},			// Values to use if no temp compensation data available
	{1658,0,0},			// Based on average values for 20 sample boards
	{1658,0,0}
};

void
AP_IMU_INS::init( Start_style style,
                     void (*delay_cb)(unsigned long t),
                     AP_PeriodicProcess * scheduler )
{
    _ins->init(scheduler);
    // if we are warm-starting, load the calibration data from EEPROM and go
    //
    if (WARM_START == style) {
        _sensor_cal.load();
    } else {

        // do cold-start calibration for both accel and gyro
        _init_gyro(delay_cb);
        _init_accel(delay_cb);

        // save calibration
        _sensor_cal.save();
    }
}

/**************************************************/

void
AP_IMU_INS::init_gyro(void (*delay_cb)(unsigned long t))
{
    _init_gyro(delay_cb);
    _sensor_cal.save();
}

void
AP_IMU_INS::_init_gyro(void (*delay_cb)(unsigned long t))
{
	int flashcount = 0;
	int tc_temp;
	float adc_in;
	float prev[3] = {0,0,0};
	float total_change;
	float max_offset;
  float ins_gyro[6];

	// cold start
	tc_temp = _ins->temperature();
 	delay_cb(500);
	Serial.printf_P(PSTR("Init Gyro"));

	for(int c = 0; c < 25; c++){
    // Mostly we are just flashing the LED's here
    // to tell the user to keep the IMU still
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		delay_cb(20);

        _ins->update();
        _ins->get_gyros(ins_gyro);

		digitalWrite(A_LED_PIN, HIGH);
		digitalWrite(C_LED_PIN, LOW);
		delay_cb(20);
	}

	for (int j = 0; j <= 2; j++)
	    _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {

    _ins->update();
    _ins->get_gyros(ins_gyro);

		for (int j = 0; j <= 2; j++){
			prev[j]     = _sensor_cal[j];
			adc_in      = ins_gyro[j];
			adc_in     -= _sensor_temp_compensation(j, tc_temp);
			_sensor_cal[j]	= adc_in;
		}

		for(int i = 0; i < 50; i++){
      
      _ins->update();
      _ins->get_gyros(ins_gyro);

			for (int j = 0; j < 3; j++){
				adc_in = ins_gyro[j];
				// Subtract temp compensated typical gyro bias
				adc_in -= _sensor_temp_compensation(j, tc_temp);
				// filter
				_sensor_cal[j] = _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			delay_cb(20);
			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
				digitalWrite(A_LED_PIN, LOW);
				digitalWrite(C_LED_PIN, HIGH);
			}

			if(flashcount >= 10) {
				flashcount = 0;
				digitalWrite(C_LED_PIN, LOW);
			    digitalWrite(A_LED_PIN, HIGH);
			}
			flashcount++;
		}

		total_change    = fabs(prev[0] - _sensor_cal[0]) + fabs(prev[1] - _sensor_cal[1]) +fabs(prev[2] - _sensor_cal[2]);
		max_offset      = (_sensor_cal[0] > _sensor_cal[1]) ? _sensor_cal[0] : _sensor_cal[1];
		max_offset      = (max_offset > _sensor_cal[2]) ? max_offset : _sensor_cal[2];
		delay_cb(500);
	} while (  total_change > _gyro_total_cal_change || max_offset > _gyro_max_cal_offset);
}

void
AP_IMU_INS::save()
{
    _sensor_cal.save();
}

void
AP_IMU_INS::init_accel(void (*delay_cb)(unsigned long t))
{
    _init_accel(delay_cb);
    _sensor_cal.save();
}

void
AP_IMU_INS::_init_accel(void (*delay_cb)(unsigned long t))
{
	int flashcount = 0;
	float adc_in;
	float prev[6] = {0,0,0};
	float total_change;
	float max_offset;
  float ins_accel[3];


	// cold start
	delay_cb(500);

	Serial.printf_P(PSTR("Init Accel"));

	for (int j=3; j<=5; j++) _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {
    _ins->update();
    _ins->get_accels(ins_accel);

		for (int j = 3; j <= 5; j++){
			prev[j] = _sensor_cal[j];
			adc_in 		    = ins_accel[j-3];
			adc_in 		    -= _sensor_temp_compensation(j, 0);  //  temperature ignored
			_sensor_cal[j]	= adc_in;
		}

		for(int i = 0; i < 50; i++){		// We take some readings...

			delay_cb(20);
      _ins->update();
      _ins->get_accels(ins_accel);

			for (int j = 3; j < 6; j++){
				adc_in 	    	= ins_accel[j-3];
				adc_in 		    -= _sensor_temp_compensation(j, 0);  //  temperature ignored
				_sensor_cal[j]	= _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
				digitalWrite(A_LED_PIN, LOW);
				digitalWrite(C_LED_PIN, HIGH);
			}

			if(flashcount >= 10) {
				flashcount = 0;
				digitalWrite(C_LED_PIN, LOW);
				digitalWrite(A_LED_PIN, HIGH);
			}
			flashcount++;
		}

		// null gravity from the Z accel
		_sensor_cal[5] += 9.805;

		total_change = fabs(prev[3] - _sensor_cal[3]) + fabs(prev[4] - _sensor_cal[4]) +fabs(prev[5] - _sensor_cal[5]);
		max_offset = (_sensor_cal[3] > _sensor_cal[4]) ? _sensor_cal[3] : _sensor_cal[4];
		max_offset = (max_offset > _sensor_cal[5]) ? max_offset : _sensor_cal[5];

		delay_cb(500);
	} while (  total_change > _accel_total_cal_change || max_offset > _accel_max_cal_offset);

	Serial.printf_P(PSTR(" "));
}

/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------

float
AP_IMU_INS::_sensor_temp_compensation(uint8_t channel, int temperature) const
{
    // do gyro temperature compensation
    if (channel < 3) {
      /* Temp comp is not implemented - return 0*/
		  return 0.0;
       /*
        return  _gyro_temp_curve[channel][0] +
                _gyro_temp_curve[channel][1] * temperature +
                _gyro_temp_curve[channel][2] * temperature * temperature;
       //*/
    }

    // Not implemented for accelerometer
    return 0.0;
}

float
AP_IMU_INS::_calibrated(uint8_t channel, float ins_value, int temperature)
{
    return ins_value - _sensor_temp_compensation(channel, temperature) - _sensor_cal[channel];
}


bool
AP_IMU_INS::update(void)
{
  float gyros[3];
  float accels[3];
  int tc_temp;

  _ins->update();
  _ins->get_gyros(gyros);
  _ins->get_accels(accels);
  _sample_time = _ins->sample_time();
	tc_temp = (int) _ins->temperature();

	// convert corrected gyro readings to delta acceleration
	//
	_gyro.x = _calibrated(0, gyros[0], tc_temp);
	_gyro.y = _calibrated(1, gyros[1], tc_temp);
	_gyro.z = _calibrated(2, gyros[2], tc_temp);

	// convert corrected accelerometer readings to acceleration
	//
	_accel.x = _calibrated(3, accels[0], tc_temp);
	_accel.y = _calibrated(4, accels[1], tc_temp);
	_accel.z = _calibrated(5, accels[2], tc_temp);

	_accel_filtered.x = _accel_filtered.x / 2 + _accel.x / 2;
	_accel_filtered.y = _accel_filtered.y / 2 + _accel.y / 2;
	_accel_filtered.z = _accel_filtered.z / 2 + _accel.z / 2;

	// always updated
	return true;
}
