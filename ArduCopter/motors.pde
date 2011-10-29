/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY 10	// one second
#define DISARM_DELAY 10	// one second
#define LEVEL_DELAY 70 // twelve seconds
#define AUTO_LEVEL_DELAY 90 // twentyfive seconds


// called at 10hz
static void arm_motors()
{
	static int arming_counter;

	// Arm motor output : Throttle down and full yaw right for more than 2 seconds
	if (g.rc_3.control_in == 0){

		// full right
		if (g.rc_4.control_in > 4000) {

			// don't allow arming in anything but manual
			if (control_mode < ALT_HOLD){

				if (arming_counter > AUTO_LEVEL_DELAY){
					auto_level_counter = 155;
					arming_counter = 0;

				}else if (arming_counter == ARM_DELAY){
					#if HIL_MODE != HIL_MODE_DISABLED
                    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
					#endif
					motor_armed 	= true;
					arming_counter 	= ARM_DELAY;

					#if PIEZO_ARMING == 1
					piezo_beep();
					piezo_beep();
					#endif

					// Tune down DCM
					// -------------------
					#if HIL_MODE != HIL_MODE_ATTITUDE
						dcm.kp_roll_pitch(0.030000);
						dcm.ki_roll_pitch(0.00001278),	// 50 hz I term
						//dcm.ki_roll_pitch(0.000006);
					#endif

					// tune down compass
					// -----------------
					dcm.kp_yaw(0.08);
					dcm.ki_yaw(0);

					// Remember Orientation
					// --------------------
					init_simple_bearing();

					// Reset home position
					// ----------------------
					if(home_is_set)
						init_home();

					if(did_ground_start == false){
						did_ground_start = true;
						startup_ground();
					}

					#if HIL_MODE != HIL_MODE_ATTITUDE
						// read Baro pressure at ground -
						// this resets Baro for more accuracy
						//-----------------------------------
						init_barometer();
					#endif

					// temp hack
					motor_light = true;
					digitalWrite(A_LED_PIN, LED_ON);

					arming_counter++;
				} else{
					arming_counter++;
				}
			}

		// full left
		}else if (g.rc_4.control_in < -4000) {
			//Serial.print(arming_counter, DEC);
			if (arming_counter > LEVEL_DELAY){
				//Serial.print("init");
				imu.init_accel(mavlink_delay);
				arming_counter = 0;

			}else if (arming_counter == DISARM_DELAY){
				#if HIL_MODE != HIL_MODE_DISABLED
                gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
				#endif

				motor_armed 	= false;
				arming_counter 	= DISARM_DELAY;
				compass.save_offsets();

				#if PIEZO_ARMING == 1
				piezo_beep();
				#endif

				// Tune down DCM
				// -------------------
				#if HIL_MODE != HIL_MODE_ATTITUDE
				//dcm.kp_roll_pitch(0.12);			// higher for fast recovery
				//dcm.ki_roll_pitch(0.00000319); 	// 1/4 of the normal rate for 200 hz loop
				#endif

				// tune up compass
				// -----------------
				dcm.kp_yaw(0.8);
				dcm.ki_yaw(0.00004);

				// Clear throttle slew
				// -------------------
				//throttle_slew = 0;

				arming_counter++;

			}else{
				arming_counter++;
			}
		// centered
		}else{
			arming_counter = 0;
		}
	}else{
		arming_counter = 0;
	}
}


/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void
set_servos_4()
{
	if (motor_armed == true && motor_auto_armed == true) {
		// creates the radio_out and pwm_out values
		output_motors_armed();
	} else{
		output_motors_disarmed();
	}
}
