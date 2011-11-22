/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "WProgram.h"
#include "ArduCopter_Motors_Tri.h"

void ArduCopter_Motors_Tri::init_out()
{
	#if INSTANT_PWM == 0
    _apm_rc->SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_4 );
	#endif
}


void ArduCopter_Motors_Tri::output_armed()
{
	int out_min = _rc_3->radio_min;
	int out_max = _rc_3->radio_max;

	// Throttle is 0 to 1000 only
	_rc_3->servo_out 	= constrain(_rc_3->servo_out, 0, 1000);

	if(_rc_3->servo_out > 0)
		out_min = _rc_3->radio_min + AC_MOTORS_MINIMUM_THROTTLE;

	_rc_1->calc_pwm();
	_rc_2->calc_pwm();
	_rc_3->calc_pwm();

	int roll_out 		= (float)_rc_1->pwm_out * .866;
	int pitch_out 		= _rc_2->pwm_out / 2;

	//left front
	_motor_out[CH_2]		= _rc_3->radio_out + roll_out + pitch_out;
	//right front
	_motor_out[CH_1]		= _rc_3->radio_out - roll_out + pitch_out;
	// rear
	_motor_out[CH_4] 	= _rc_3->radio_out - _rc_2->pwm_out;

	//_motor_out[CH_4]		+= (float)(abs(_rc_4->control_in)) * .013;

	// Tridge's stability patch
	if (_motor_out[CH_1] > out_max) {
		_motor_out[CH_2] -= (_motor_out[CH_1] - out_max) >> 1;
		_motor_out[CH_4] -= (_motor_out[CH_1] - out_max) >> 1;
		_motor_out[CH_1] = out_max;
	}

	if (_motor_out[CH_2] > out_max) {
		_motor_out[CH_1] -= (_motor_out[CH_2] - out_max) >> 1;
		_motor_out[CH_4] -= (_motor_out[CH_2] - out_max) >> 1;
		_motor_out[CH_2] = out_max;
	}

	if (_motor_out[CH_4] > out_max) {
		_motor_out[CH_1] -= (_motor_out[CH_4] - out_max) >> 1;
		_motor_out[CH_2] -= (_motor_out[CH_4] - out_max) >> 1;
		_motor_out[CH_4] = out_max;
	}

	// limit output so motors don't stop
	_motor_out[CH_1]		= max(_motor_out[CH_1], 	out_min);
	_motor_out[CH_2]		= max(_motor_out[CH_2], 	out_min);
	_motor_out[CH_4] 	= max(_motor_out[CH_4], 	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(_rc_3->servo_out == 0){
		_motor_out[CH_1]		= _rc_3->radio_min;
		_motor_out[CH_2]		= _rc_3->radio_min;
		_motor_out[CH_4] 	= _rc_3->radio_min;
	}
	#endif

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);

	#if INSTANT_PWM == 1
	// InstantPWM
	_apm_rc->Force_Out0_Out1();
	_apm_rc->Force_Out2_Out3();
	#endif
}

void ArduCopter_Motors_Tri::output_disarmed()
{
	if(_rc_3->control_in > 0){
		// we have pushed up the throttle
		// remove safety
		*_motor_auto_armed = true;
	}

	// fill the _motor_out[] array for HIL use
	for (unsigned char i = 0; i < 8; i++) {
		_motor_out[i] = _rc_3->radio_min;
	}

	// Send commands to motors
	_apm_rc->OutputCh(CH_1, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_2, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_4, _rc_3->radio_min);
}

void ArduCopter_Motors_Tri::output_test()
{
	_motor_out[CH_1] = _rc_3->radio_min;
	_motor_out[CH_2] = _rc_3->radio_min;
	_motor_out[CH_4] = _rc_3->radio_min;


	if(_rc_1->control_in > 3000){	// right
		_motor_out[CH_1] += 100;
	}

	if(_rc_1->control_in < -3000){	// left
		_motor_out[CH_2] += 100;
	}

	if(_rc_2->control_in > 3000){	// back
		_motor_out[CH_4] += 100;
	}

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);
}

