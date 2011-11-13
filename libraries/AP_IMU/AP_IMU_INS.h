// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_IMU_INS.h
/// @brief	IMU driver on top of an AP_InertialSensor (INS) driver.
//          Provides offset calibration for for the gyro and accel.

#ifndef __AP_IMU_INS_H__
#define __AP_IMU_INS_H__


#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include <inttypes.h>

#include "IMU.h"

class AP_IMU_INS : public IMU
{

public:
    /// Constructor
    ///
    /// Saves the ADC pointer and constructs the calibration data variable.
    ///
    /// @param  adc         Pointer to the AP_ADC instance that is connected to the gyro and accelerometer.
    /// @param  key         The AP_Var::key value we will use when loading/saving calibration data.
    ///
	AP_IMU_INS(AP_InertialSensor *ins, AP_Var::Key key) :
        _ins(ins),
	    _sensor_cal(key, PSTR("IMU_SENSOR_CAL"))
	{}

	/// Do warm or cold start.
	///
	/// @note   For a partial-warmstart where e.g. the accelerometer calibration should be preserved
	///         but the gyro cal needs to be re-performed, start with ::init(WARM_START) to load the
	///         previous calibration settings, then force a re-calibration of the gyro with ::init_gyro.
	///
	/// @param  style   Selects the initialisation style.
	///                 COLD_START performs calibration of both the accelerometer and gyro.
	///                 WARM_START loads accelerometer and gyro calibration from a previous cold start.
	///
	virtual void		init( Start_style style = COLD_START,
                              void (*delay_cb)(unsigned long t) = delay,
                              AP_PeriodicProcess *scheduler = NULL );

	virtual void		save();
	virtual void		init_accel(void (*delay_cb)(unsigned long t) = delay);
	virtual void		init_gyro(void (*delay_cb)(unsigned long t) = delay);
	virtual bool		update(void);

	// for jason
	virtual float		gx()				{ return _sensor_cal[0]; }
	virtual float		gy()				{ return _sensor_cal[1]; }
	virtual float		gz()				{ return _sensor_cal[2]; }
	virtual float		ax()				{ return _sensor_cal[3]; }
	virtual float		ay()				{ return _sensor_cal[4]; }
	virtual float		az()				{ return _sensor_cal[5]; }

	virtual void		ax(const float v)		{ _sensor_cal[3] = v; }
	virtual void		ay(const float v)		{ _sensor_cal[4] = v; }
	virtual void		az(const float v)		{ _sensor_cal[5] = v; }


private:
    AP_InertialSensor   *_ins;          ///< INS provides an axis and unit correct sensor source.
    AP_VarA<float,6>    _sensor_cal;    ///< Calibrated sensor offsets

    virtual void        _init_accel(void (*delay_cb)(unsigned long t));  ///< no-save implementation
    virtual void        _init_gyro(void (*delay_cb)(unsigned long t));   ///< no-save implementation

    float _calibrated(uint8_t channel, float ins_value);

	// Gyro and Accelerometer calibration criterial
	//
	static const float		_gyro_total_cal_change = 4.0;		// Experimentally derived - allows for some minor motion
	static const float		_gyro_max_cal_offset = 320.0;
	static const float		_accel_total_cal_change = 4.0;
	static const float		_accel_max_cal_offset = 250.0;

};

#endif
