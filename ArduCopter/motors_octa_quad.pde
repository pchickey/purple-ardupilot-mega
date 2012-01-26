/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	OCTA_QUAD_FRAME

static void init_motors_out()
{
	#if INSTANT_PWM == 0
    APM_RC.SetFastOutputChannels( _BV(MOT_1) | _BV(MOT_2) | _BV(MOT_3) | _BV(MOT_4)
                                | _BV(MOT_5) | _BV(MOT_6) | _BV(MOT_7) | _BV(MOT_8) );
	#endif
}

static void motors_output_enable()
{
  APM_RC.enable_out(MOT_1);
  APM_RC.enable_out(MOT_2);
  APM_RC.enable_out(MOT_3);
  APM_RC.enable_out(MOT_4);
  APM_RC.enable_out(MOT_5);
  APM_RC.enable_out(MOT_6);
  APM_RC.enable_out(MOT_7);
  APM_RC.enable_out(MOT_8);
}

static void output_motors_armed()
{
	int roll_out, pitch_out;
	int out_min = g.rc_3.radio_min;
	int out_max = g.rc_3.radio_max;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, 800);

	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + MINIMUM_THROTTLE;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	if(g.frame_orientation == X_FRAME){
		roll_out 	 	= (float)g.rc_1.pwm_out * .707;
		pitch_out 	 	= (float)g.rc_2.pwm_out * .707;

        // Front Left
		motor_out[MOT_5]    = ((g.rc_3.radio_out * g.top_bottom_ratio) + roll_out + pitch_out);  // CCW TOP
	    motor_out[MOT_6]	   =  g.rc_3.radio_out + roll_out + pitch_out;			        // CW

        // Front Right
		motor_out[MOT_7]	= ((g.rc_3.radio_out * g.top_bottom_ratio) - roll_out + pitch_out);	// CCW TOP
	    motor_out[MOT_8]	=  g.rc_3.radio_out - roll_out + pitch_out;			        // CW

		// Back Left
		motor_out[MOT_3]		= ((g.rc_3.radio_out * g.top_bottom_ratio) + roll_out - pitch_out);	// CCW TOP
	    motor_out[MOT_4]		=  g.rc_3.radio_out + roll_out - pitch_out;			        // CW

		// Back Right
		motor_out[MOT_1]		= ((g.rc_3.radio_out * g.top_bottom_ratio) - roll_out - pitch_out);	// CCW TOP
	    motor_out[MOT_2]		=  g.rc_3.radio_out - roll_out - pitch_out;			        // CW



	}if(g.frame_orientation == PLUS_FRAME){
		roll_out 	 	= g.rc_1.pwm_out;
		pitch_out 	 	= g.rc_2.pwm_out;

		 // Left
		motor_out[MOT_5]    = (g.rc_3.radio_out * g.top_bottom_ratio) - roll_out;   // CCW TOP
	    motor_out[MOT_6]		=  g.rc_3.radio_out - roll_out;			        // CW

        // Right
		motor_out[MOT_1]		= (g.rc_3.radio_out * g.top_bottom_ratio) + roll_out;	// CCW TOP
	    motor_out[MOT_2]		=  g.rc_3.radio_out + roll_out;		            // CW

		// Front
		motor_out[MOT_7]		= (g.rc_3.radio_out * g.top_bottom_ratio) + pitch_out;	// CCW TOP
	    motor_out[MOT_8]		=  g.rc_3.radio_out + pitch_out;			    // CW

		// Back
		motor_out[MOT_3]		= (g.rc_3.radio_out * g.top_bottom_ratio) - pitch_out;	// CCW TOP
	    motor_out[MOT_4]		=  g.rc_3.radio_out - pitch_out;			    // CW

	}

	// Yaw
	motor_out[MOT_1]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_3]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_5] 	+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_7] 	+= g.rc_4.pwm_out;	// CCW

	motor_out[MOT_2]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_4]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_6]	    -= g.rc_4.pwm_out;	// CW
	motor_out[MOT_8]    -= g.rc_4.pwm_out;	// CW

	// TODO add stability patch
	motor_out[MOT_1]		= min(motor_out[MOT_1], 	out_max);
	motor_out[MOT_2]		= min(motor_out[MOT_2], 	out_max);
	motor_out[MOT_3]		= min(motor_out[MOT_3], 	out_max);
	motor_out[MOT_4]		= min(motor_out[MOT_4], 	out_max);
	motor_out[MOT_5]		= min(motor_out[MOT_5],  out_max);
	motor_out[MOT_6]		= min(motor_out[MOT_6],  out_max);
	motor_out[MOT_7]	= min(motor_out[MOT_7], out_max);
	motor_out[MOT_8] 	= min(motor_out[MOT_8], out_max);

	// limit output so motors don't stop
	motor_out[MOT_1]		= max(motor_out[MOT_1], 	out_min);
	motor_out[MOT_2]		= max(motor_out[MOT_2], 	out_min);
	motor_out[MOT_3]		= max(motor_out[MOT_3], 	out_min);
	motor_out[MOT_4] 	= max(motor_out[MOT_4], 	out_min);
	motor_out[MOT_5]		= max(motor_out[MOT_5], 	out_min);
	motor_out[MOT_6] 	= max(motor_out[MOT_6], 	out_min);
	motor_out[MOT_7]	= max(motor_out[MOT_7], out_min);
	motor_out[MOT_8] 	= max(motor_out[MOT_8], out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		motor_out[MOT_1]		= g.rc_3.radio_min;
		motor_out[MOT_2]		= g.rc_3.radio_min;
		motor_out[MOT_3]		= g.rc_3.radio_min;
		motor_out[MOT_4] 	= g.rc_3.radio_min;
		motor_out[MOT_5] 	= g.rc_3.radio_min;
		motor_out[MOT_6] 	= g.rc_3.radio_min;
		motor_out[MOT_7] 	= g.rc_3.radio_min;
		motor_out[MOT_8] 	= g.rc_3.radio_min;
	}
	#endif

	// this filter slows the acceleration of motors vs the deceleration
	// Idea by Denny Rowland to help with his Yaw issue
	for(int8_t m = 0; i <= 8; m++ ) {
    int i = mot_to_ch(m);
		if(motor_filtered[i] < motor_out[i]){
			motor_filtered[i] = (motor_out[i] + motor_filtered[i]) / 2;
		}else{
			// don't filter
			motor_filtered[i] = motor_out[i];
		}
	}

	APM_RC.OutputCh(MOT_1, motor_filtered[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_filtered[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_filtered[MOT_3]);
	APM_RC.OutputCh(MOT_4, motor_filtered[MOT_4]);
	APM_RC.OutputCh(MOT_5, motor_filtered[MOT_5]);
	APM_RC.OutputCh(MOT_6, motor_filtered[MOT_6]);
	APM_RC.OutputCh(MOT_7, motor_filtered[MOT_7]);
	APM_RC.OutputCh(MOT_8, motor_filtered[MOT_8]);

	#if INSTANT_PWM == 1
	// InstantPWM
	APM_RC.Force_Out0_Out1();
	APM_RC.Force_Out2_Out3();
	APM_RC.Force_Out6_Out7();
	#endif
}

static void output_motors_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle
		// remove safety
		motor_auto_armed = true;
	}

	// fill the motor_out[] array for HIL use
	for (unsigned char i = 0; i < 11; i++) {
		motor_out[i] = g.rc_3.radio_min;
	}

	// Send commands to motors
	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_7, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_8, g.rc_3.radio_min);
}

static void output_motor_test()
{
	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_7, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_7, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_8, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_8, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min + 100);
	delay(1000);

	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min + 100);
	delay(1000);
}

#endif
