/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "WProgram.h"
#include "ArduCopter_Motors_Octa_Quad.h"

void ArduCopter_Motors_Octa_Quad::init_out()
{
	#if INSTANT_PWM == 0
    _apm_rc->SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_3 | MSK_CH_4
                                | MSK_CH_7 | MSK_CH_8 | MSK_CH_10 | MSK_CH_11 );
	#endif
}

void ArduCopter_Motors_Octa_Quad::output_armed()
{
	int roll_out, pitch_out;
	int out_min = g.rc_3.radio_min;
	int out_max = g.rc_3.radio_max;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, 1000);

	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + AC_MOTORS_MINIMUM_THROTTLE;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	if(g.frame_orientation == X_FRAME){
		roll_out 	 	= (float)g.rc_1.pwm_out * .707;
		pitch_out 	 	= (float)g.rc_2.pwm_out * .707;

        // Front Left
		_motor_out[CH_7]    = ((g.rc_3.radio_out * g.top_bottom_ratio) + roll_out + pitch_out);  // CCW TOP
	    _motor_out[CH_8]	   =  g.rc_3.radio_out + roll_out + pitch_out;			        // CW

        // Front Right
		_motor_out[CH_10]	= ((g.rc_3.radio_out * g.top_bottom_ratio) - roll_out + pitch_out);	// CCW TOP
	    _motor_out[CH_11]	=  g.rc_3.radio_out - roll_out + pitch_out;			        // CW

		// Back Left
		_motor_out[CH_3]		= ((g.rc_3.radio_out * g.top_bottom_ratio) + roll_out - pitch_out);	// CCW TOP
	    _motor_out[CH_4]		=  g.rc_3.radio_out + roll_out - pitch_out;			        // CW

		// Back Right
		_motor_out[CH_1]		= ((g.rc_3.radio_out * g.top_bottom_ratio) - roll_out - pitch_out);	// CCW TOP
	    _motor_out[CH_2]		=  g.rc_3.radio_out - roll_out - pitch_out;			        // CW



	}if(g.frame_orientation == PLUS_FRAME){
		roll_out 	 	= g.rc_1.pwm_out;
		pitch_out 	 	= g.rc_2.pwm_out;

		 // Left
		_motor_out[CH_7]    = (g.rc_3.radio_out * g.top_bottom_ratio) - roll_out;   // CCW TOP
	    _motor_out[CH_8]		=  g.rc_3.radio_out - roll_out;			        // CW

        // Right
		_motor_out[CH_1]		= (g.rc_3.radio_out * g.top_bottom_ratio) + roll_out;	// CCW TOP
	    _motor_out[CH_2]		=  g.rc_3.radio_out + roll_out;		            // CW

		// Front
		_motor_out[CH_10]		= (g.rc_3.radio_out * g.top_bottom_ratio) + pitch_out;	// CCW TOP
	    _motor_out[CH_11]		=  g.rc_3.radio_out + pitch_out;			    // CW

		// Back
		_motor_out[CH_3]		= (g.rc_3.radio_out * g.top_bottom_ratio) - pitch_out;	// CCW TOP
	    _motor_out[CH_4]		=  g.rc_3.radio_out - pitch_out;			    // CW

	}

	// Yaw
	_motor_out[CH_1]		+= g.rc_4.pwm_out;	// CCW
	_motor_out[CH_3]		+= g.rc_4.pwm_out;	// CCW
	_motor_out[CH_7] 	+= g.rc_4.pwm_out;	// CCW
	_motor_out[CH_10] 	+= g.rc_4.pwm_out;	// CCW

	_motor_out[CH_2]		-= g.rc_4.pwm_out;	// CW
	_motor_out[CH_4]		-= g.rc_4.pwm_out;	// CW
	_motor_out[CH_8]	    -= g.rc_4.pwm_out;	// CW
	_motor_out[CH_11]    -= g.rc_4.pwm_out;	// CW

	// TODO add stability patch
	_motor_out[CH_1]		= min(_motor_out[CH_1], 	out_max);
	_motor_out[CH_2]		= min(_motor_out[CH_2], 	out_max);
	_motor_out[CH_3]		= min(_motor_out[CH_3], 	out_max);
	_motor_out[CH_4]		= min(_motor_out[CH_4], 	out_max);
	_motor_out[CH_7]		= min(_motor_out[CH_7],  out_max);
	_motor_out[CH_8]		= min(_motor_out[CH_8],  out_max);
	_motor_out[CH_10]	= min(_motor_out[CH_10], out_max);
	_motor_out[CH_11] 	= min(_motor_out[CH_11], out_max);

	// limit output so motors don't stop
	_motor_out[CH_1]		= max(_motor_out[CH_1], 	out_min);
	_motor_out[CH_2]		= max(_motor_out[CH_2], 	out_min);
	_motor_out[CH_3]		= max(_motor_out[CH_3], 	out_min);
	_motor_out[CH_4] 	= max(_motor_out[CH_4], 	out_min);
	_motor_out[CH_7]		= max(_motor_out[CH_7], 	out_min);
	_motor_out[CH_8] 	= max(_motor_out[CH_8], 	out_min);
	_motor_out[CH_10]	= max(_motor_out[CH_10], out_min);
	_motor_out[CH_11] 	= max(_motor_out[CH_11], out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		_motor_out[CH_1]		= g.rc_3.radio_min;
		_motor_out[CH_2]		= g.rc_3.radio_min;
		_motor_out[CH_3]		= g.rc_3.radio_min;
		_motor_out[CH_4] 	= g.rc_3.radio_min;
		_motor_out[CH_7] 	= g.rc_3.radio_min;
		_motor_out[CH_8] 	= g.rc_3.radio_min;
		_motor_out[CH_10] 	= g.rc_3.radio_min;
		_motor_out[CH_11] 	= g.rc_3.radio_min;
	}
	#endif

	_apm_rc->OutputCh(CH_1, _motor_out[CH_1]);
	_apm_rc->OutputCh(CH_2, _motor_out[CH_2]);
	_apm_rc->OutputCh(CH_3, _motor_out[CH_3]);
	_apm_rc->OutputCh(CH_4, _motor_out[CH_4]);
	_apm_rc->OutputCh(CH_7, _motor_out[CH_7]);
	_apm_rc->OutputCh(CH_8, _motor_out[CH_8]);
	_apm_rc->OutputCh(CH_10, _motor_out[CH_10]);
	_apm_rc->OutputCh(CH_11, _motor_out[CH_11]);

	#if INSTANT_PWM == 1
	// InstantPWM
	_apm_rc->Force_Out0_Out1();
	_apm_rc->Force_Out2_Out3();
	_apm_rc->Force_Out6_Out7();
	#endif
}

void ArduCopter_Motors_Octa_Quad::output_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle
		// remove safety
		motor_auto_armed = true;
	}

	// fill the _motor_out[] array for HIL use
	for (unsigned char i = 0; i < 11; i++) {
		_motor_out[i] = g.rc_3.radio_min;
	}

	// Send commands to motors
	_apm_rc->OutputCh(CH_1, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_2, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_3, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_4, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_7, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_8, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_10, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_11, g.rc_3.radio_min);
}

void ArduCopter_Motors_Octa_Quad::output_test()
{
	_apm_rc->OutputCh(CH_8, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_10, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_10, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_11, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_11, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_1, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_1, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_2, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_2, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_3, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_3, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_4, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_4, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_7, g.rc_3.radio_min + 100);
	delay(1000);

	_apm_rc->OutputCh(CH_7, g.rc_3.radio_min);
	_apm_rc->OutputCh(CH_8, g.rc_3.radio_min + 100);
	delay(1000);
}

