/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "WProgram.h"
#include "ArduCopter_Motors_Quad.h"

void ArduCopter_Motors_Quad::init_out()
{
	#if INSTANT_PWM == 0
    _apm_rc->SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_3 | MSK_CH_4 );
	#endif
}

void ArduCopter_Motors_Quad::output_armed()
{
	int roll_out, pitch_out;
	int out_min = _rc_3->radio_min;
	int out_max = _rc_3->radio_max;

	// Throttle is 0 to 1000 only
	_rc_3->servo_out 	= constrain(_rc_3->servo_out, 0, 1000);

	if(_rc_3->servo_out > 0)
		out_min = _rc_3->radio_min + AC_MOTORS_MINIMUM_THROTTLE;

	_rc_1->calc_pwm();
	_rc_2->calc_pwm();
	_rc_3->calc_pwm();
	_rc_4->calc_pwm();

	if(_frame_orientation == AC_MOTORS_X_FRAME){
		roll_out 	 	= _rc_1->pwm_out * .707;
		pitch_out 	 	= _rc_2->pwm_out * .707;

		// left
		_motor_out[CH_3]	 	= _rc_3->radio_out + roll_out + pitch_out;	// FRONT
		_motor_out[CH_2]	 	= _rc_3->radio_out + roll_out - pitch_out;	// BACK

		// right
		_motor_out[CH_1]		= _rc_3->radio_out - roll_out + pitch_out;  // FRONT
		_motor_out[CH_4] 	= _rc_3->radio_out - roll_out - pitch_out;	// BACK

	}else{

		roll_out 	 	= _rc_1->pwm_out;
		pitch_out 	 	= _rc_2->pwm_out;

		// left
		_motor_out[CH_1]		= _rc_3->radio_out - roll_out;
		// right
		_motor_out[CH_2]		= _rc_3->radio_out + roll_out;
		// front
		_motor_out[CH_3]		= _rc_3->radio_out + pitch_out;
		// back
		_motor_out[CH_4] 	= _rc_3->radio_out - pitch_out;
	}

	// Yaw input
	_motor_out[CH_1]		+=  _rc_4->pwm_out; 	// CCW
	_motor_out[CH_2]		+=  _rc_4->pwm_out; 	// CCW
	_motor_out[CH_3]		-=  _rc_4->pwm_out; 	// CW
	_motor_out[CH_4] 	-=  _rc_4->pwm_out; 	// CW

    /* We need to clip motor output at out_max. When cipping a motors
     * output we also need to compensate for the instability by
     * lowering the opposite motor by the same proportion. This
     * ensures that we retain control when one or more of the motors
     * is at its maximum output
     */
    for (int i=CH_1; i<=CH_4; i++) {
        if (_motor_out[i] > out_max) {
            // note that i^1 is the opposite motor
            _motor_out[i^1] -= _motor_out[i] - out_max;
            _motor_out[i] = out_max;
        }
    }

	// limit output so motors don't stop
	_motor_out[CH_1]		= max(_motor_out[CH_1], 	out_min);
	_motor_out[CH_2]		= max(_motor_out[CH_2], 	out_min);
	_motor_out[CH_3]		= max(_motor_out[CH_3], 	out_min);
	_motor_out[CH_4] 	= max(_motor_out[CH_4], 	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(_rc_3->servo_out == 0){
		_motor_out[CH_1]		= _rc_3->radio_min;
		_motor_out[CH_2]		= _rc_3->radio_min;
		_motor_out[CH_3]		= _rc_3->radio_min;
		_motor_out[CH_4] 	= _rc_3->radio_min;
	}
	#endif

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_3, _motor_out[CH_3]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);

	#if INSTANT_PWM == 1
	// InstantPWM
	_apm_rc->Force_Out0_Out1();
	_apm_rc->Force_Out2_Out3();
	#endif
}

void ArduCopter_Motors_Quad::output_disarmed()
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
	_apm_rc->OutputCh(CH_3, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_4, _rc_3->radio_min);

	// InstantPWM
	_apm_rc->Force_Out0_Out1();
	_apm_rc->Force_Out2_Out3();
}

/*
static void debug_motors()
{
	Serial.printf("1:%d\t2:%d\t3:%d\t4:%d\n",
				_motor_out[CH_1],
				_motor_out[CH_2],
				_motor_out[CH_3],
				_motor_out[CH_4]);
}
*/

void ArduCopter_Motors_Quad::output_test()
{
	_motor_out[CH_1] = _rc_3->radio_min;
	_motor_out[CH_2] = _rc_3->radio_min;
	_motor_out[CH_3] = _rc_3->radio_min;
	_motor_out[CH_4] = _rc_3->radio_min;


	if(_frame_orientation == AC_MOTORS_X_FRAME){
//  31
//	24
		if(_rc_1->control_in > 3000){
			_motor_out[CH_1] += 100;
			_motor_out[CH_4] += 100;
		}

		if(_rc_1->control_in < -3000){
			_motor_out[CH_2] += 100;
			_motor_out[CH_3] += 100;
		}

		if(_rc_2->control_in > 3000){
			_motor_out[CH_2] += 100;
			_motor_out[CH_4] += 100;
		}

		if(_rc_2->control_in < -3000){
			_motor_out[CH_1] += 100;
			_motor_out[CH_3] += 100;
		}

	}else{
//  3
// 2 1
//	4
		if(_rc_1->control_in > 3000)
			_motor_out[CH_1] += 100;

		if(_rc_1->control_in < -3000)
			_motor_out[CH_2] += 100;

		if(_rc_2->control_in > 3000)
			_motor_out[CH_4] += 100;

		if(_rc_2->control_in < -3000)
			_motor_out[CH_3] += 100;
	}

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_3, _motor_out[CH_3]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);
}

