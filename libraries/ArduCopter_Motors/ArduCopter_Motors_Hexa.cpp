/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "WProgram.h"
#include "ArduCopter_Motors_Hexa.h"

void ArduCopter_Motors_Hexa::init_out()
{
	#if INSTANT_PWM == 0
    _apm_rc->SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_3 | MSK_CH_4
                                | MSK_CH_7 | MSK_CH_8 );
	#endif
}

void ArduCopter_Motors_Hexa::output_armed()
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
		roll_out 	 	= _rc_1->pwm_out / 2;
		pitch_out 	 	= (float)_rc_2->pwm_out * .866;

		//left side
		_motor_out[CH_2]		= _rc_3->radio_out + _rc_1->pwm_out;		// CCW Middle
		_motor_out[CH_3]		= _rc_3->radio_out + roll_out + pitch_out;	// CW Front
		_motor_out[CH_8]     = _rc_3->radio_out + roll_out - pitch_out;	// CW Back

		//right side
		_motor_out[CH_1]		= _rc_3->radio_out - _rc_1->pwm_out;		// CW Middle
		_motor_out[CH_7] 	= _rc_3->radio_out - roll_out + pitch_out;	// CCW Front
		_motor_out[CH_4] 	= _rc_3->radio_out - roll_out - pitch_out;	// CCW Back

	}else{
		roll_out 	 	= (float)_rc_1->pwm_out * .866;
		pitch_out 	 	= _rc_2->pwm_out / 2;

		//Front side
		_motor_out[CH_1]		= _rc_3->radio_out + _rc_2->pwm_out;		// CW	 FRONT
		_motor_out[CH_7] 	= _rc_3->radio_out + roll_out + pitch_out;	// CCW	 FRONT LEFT
		_motor_out[CH_4] 	= _rc_3->radio_out - roll_out + pitch_out;	// CCW	 FRONT RIGHT

		//Back side
		_motor_out[CH_2]		= _rc_3->radio_out - _rc_2->pwm_out;		// CCW	BACK
		_motor_out[CH_3]		= _rc_3->radio_out + roll_out - pitch_out;	// CW, 	BACK LEFT
		_motor_out[CH_8]		= _rc_3->radio_out - roll_out - pitch_out;	// CW	BACK RIGHT
	}

	// Yaw
	_motor_out[CH_2]		+= _rc_4->pwm_out;	// CCW
	_motor_out[CH_7]		+= _rc_4->pwm_out;	// CCW
	_motor_out[CH_4] 	+= _rc_4->pwm_out;	// CCW

	_motor_out[CH_3]		-= _rc_4->pwm_out;	// CW
	_motor_out[CH_1]		-= _rc_4->pwm_out;	// CW
	_motor_out[CH_8]		-= _rc_4->pwm_out;  // CW


	// Tridge's stability patch
    for (int i = CH_1; i<=CH_8; i++) {
	if(i == CH_5 || i == CH_6)
		break;
        if (_motor_out[i] > out_max) {
            // note that i^1 is the opposite motor
            _motor_out[i^1] -= _motor_out[i] - out_max;
            _motor_out[i] = out_max;
        }
    }

	// limit output so motors don't stop
	_motor_out[CH_1] = max(_motor_out[CH_1],  out_min);
	_motor_out[CH_2] = max(_motor_out[CH_2],  out_min);
	_motor_out[CH_3] = max(_motor_out[CH_3],  out_min);
	_motor_out[CH_4] = max(_motor_out[CH_4],  out_min);
	_motor_out[CH_7] = max(_motor_out[CH_7],  out_min);
	_motor_out[CH_8] = max(_motor_out[CH_8],  out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(_rc_3->servo_out == 0){
		_motor_out[CH_1]		= _rc_3->radio_min;
		_motor_out[CH_2]		= _rc_3->radio_min;
		_motor_out[CH_3]		= _rc_3->radio_min;
		_motor_out[CH_4] 	= _rc_3->radio_min;
		_motor_out[CH_7] 	= _rc_3->radio_min;
		_motor_out[CH_8] 	= _rc_3->radio_min;
	}
	#endif

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_3, _motor_out[CH_3]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);
	_apm_rc->OutputCh(CH_7, _motor_out[CH_7]);
	_apm_rc->OutputCh(CH_8, _motor_out[CH_8]);

	#if INSTANT_PWM == 1
	// InstantPWM
	_apm_rc->Force_Out0_Out1();
	_apm_rc->Force_Out2_Out3();
	_apm_rc->Force_Out6_Out7();
	#endif

}

void ArduCopter_Motors_Hexa::output_disarmed()
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
	_apm_rc->OutputCh(CH_7, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_8, _rc_3->radio_min);
}

void ArduCopter_Motors_Hexa::output_test()
{
	_motor_out[CH_1] = _rc_3->radio_min;
	_motor_out[CH_2] = _rc_3->radio_min;
	_motor_out[CH_3] = _rc_3->radio_min;
	_motor_out[CH_4] = _rc_3->radio_min;
	_motor_out[CH_7] = _rc_3->radio_min;
	_motor_out[CH_8] = _rc_3->radio_min;


	if(_frame_orientation == AC_MOTORS_X_FRAME){
//  31
//	24
		if(_rc_1->control_in > 3000){	// right
			_motor_out[CH_1] += 100;
		}

		if(_rc_1->control_in < -3000){	// left
			_motor_out[CH_2] += 100;
		}

		if(_rc_2->control_in > 3000){ 	// back
			_motor_out[CH_8] += 100;
			_motor_out[CH_4] += 100;
		}

		if(_rc_2->control_in < -3000){	// front
			_motor_out[CH_7] += 100;
			_motor_out[CH_3] += 100;
		}

	}else{
//  3
// 2 1
//	4
		if(_rc_1->control_in > 3000){	// right
			_motor_out[CH_4] += 100;
			_motor_out[CH_8] += 100;
		}

		if(_rc_1->control_in < -3000){	// left
			_motor_out[CH_7] += 100;
			_motor_out[CH_3] += 100;
		}

		if(_rc_2->control_in > 3000){ 	// back
			_motor_out[CH_2] += 100;
		}

		if(_rc_2->control_in < -3000){	// front
			_motor_out[CH_1] += 100;
		}

	}

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_3, _motor_out[CH_3]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);
	_apm_rc->OutputCh(CH_7, _motor_out[CH_7]);
	_apm_rc->OutputCh(CH_8, _motor_out[CH_8]);
}

/*
	_apm_rc->OutputCh(CH_2, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_3, _rc_3->radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_3, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_7, _rc_3->radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_7, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_1, _rc_3->radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_1, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_4, _rc_3->radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_4, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_8, _rc_3->radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_8, _rc_3->radio_min);
	_apm_rc->OutputCh(CH_2, _rc_3->radio_min + 100);
	delay(1000);
}
*/

