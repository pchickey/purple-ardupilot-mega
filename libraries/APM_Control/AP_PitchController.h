// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_PITCH_CONTROLLER_H__
#define __AP_PITCH_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <math.h> // for fabs()

class AP_PitchController {
public:
	void set_ahrs(AP_AHRS *ahrs) { _ahrs = ahrs; }

	int32_t get_servo_out(int32_t angle, float scaler = 1.0, bool stabilize = false);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _kp_angle;
	AP_Float _kp_ff;
	AP_Float _kp_rate;
	AP_Float _ki_rate;
	AP_Int16 _max_rate_pos;
	AP_Int16 _max_rate_neg;
	AP_Float _roll_ff;
	AP_Float _stabilize_gain;
	uint32_t _last_t;
	float _last_rate;
	
	float _integrator;
	
	AP_AHRS *_ahrs;
	
	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	static const uint8_t _fCut = 20;
};

#endif // __AP_PITCH_CONTROLLER_H__
