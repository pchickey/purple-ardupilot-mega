/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "ArduCopter_Motors_Heli.h"

#define HELI_SERVO_AVERAGING_DIGITAL 0  // 250Hz
#define HELI_SERVO_AVERAGING_ANALOG  2  // 125Hz

static bool heli_swash_initialised = false;
static int heli_throttle_mid = 0;  // throttle mid point in pwm form (i.e. 0 ~ 1000)

// heli_servo_averaging:
//   0 or 1 = no averaging, 250hz
//   2 = average two samples, 125hz
//   3 = averaging three samples = 83.3 hz
//   4 = averaging four samples = 62.5 hz
//   5 = averaging 5 samples = 50hz
//   digital = 0 / 250hz, analog = 2 / 83.3

static void heli_init_swash()
{
    int i;

	// swash servo initialisation
	g.heli_servo_1.set_range(0,1000);
	g.heli_servo_2.set_range(0,1000);
	g.heli_servo_3.set_range(0,1000);
	g.heli_servo_4.set_angle(4500);

	// pitch factors
	heli_pitchFactor[CH_1] = cos(radians(g.heli_servo1_pos - g.heli_phase_angle));
	heli_pitchFactor[CH_2] = cos(radians(g.heli_servo2_pos - g.heli_phase_angle));
	heli_pitchFactor[CH_3] = cos(radians(g.heli_servo3_pos - g.heli_phase_angle));

	// roll factors
    heli_rollFactor[CH_1] = cos(radians(g.heli_servo1_pos + 90 - g.heli_phase_angle));
	heli_rollFactor[CH_2] = cos(radians(g.heli_servo2_pos + 90 - g.heli_phase_angle));
	heli_rollFactor[CH_3] = cos(radians(g.heli_servo3_pos + 90 - g.heli_phase_angle));

	// ensure g.heli_coll values are reasonable
	if( g.heli_coll_min >= g.heli_coll_max ) {
	    g.heli_coll_min = 1000;
		g.heli_coll_max = 2000;
	}
	g.heli_coll_mid = constrain(g.heli_coll_mid, g.heli_coll_min, g.heli_coll_max);

	// servo min/max values
	if( g.heli_servo_1.get_reverse() ) {
		g.heli_servo_1.radio_min = 3000 - g.heli_coll_max + (g.heli_servo_1.radio_trim-1500);
		g.heli_servo_1.radio_max = 3000 - g.heli_coll_min + (g.heli_servo_1.radio_trim-1500);
	}else{
		g.heli_servo_1.radio_min = g.heli_coll_min + (g.heli_servo_1.radio_trim-1500);
		g.heli_servo_1.radio_max = g.heli_coll_max + (g.heli_servo_1.radio_trim-1500);
	}
	if( g.heli_servo_2.get_reverse() ) {
		g.heli_servo_2.radio_min = 3000 - g.heli_coll_max + (g.heli_servo_2.radio_trim-1500);
		g.heli_servo_2.radio_max = 3000 - g.heli_coll_min + (g.heli_servo_2.radio_trim-1500);
	}else{
		g.heli_servo_2.radio_min = g.heli_coll_min + (g.heli_servo_2.radio_trim-1500);
		g.heli_servo_2.radio_max = g.heli_coll_max + (g.heli_servo_2.radio_trim-1500);
	}
	if( g.heli_servo_3.get_reverse() ) {
		g.heli_servo_3.radio_min = 3000 - g.heli_coll_max + (g.heli_servo_3.radio_trim-1500);
		g.heli_servo_3.radio_max = 3000 - g.heli_coll_min + (g.heli_servo_3.radio_trim-1500);
	}else{
		g.heli_servo_3.radio_min = g.heli_coll_min + (g.heli_servo_3.radio_trim-1500);
		g.heli_servo_3.radio_max = g.heli_coll_max + (g.heli_servo_3.radio_trim-1500);
	}

	// calculate throttle mid point
	heli_throttle_mid = (g.heli_coll_mid-g.heli_coll_min)*(1000.0/(g.heli_coll_max-g.heli_coll_min));

	// reset the servo averaging
	for( i=0; i<=3; i++ )
	    heli_servo_out[i] = 0;

    // double check heli_servo_averaging is reasonable
	if( g.heli_servo_averaging < 0 || g.heli_servo_averaging < 0 > 5 ) {
	    g.heli_servo_averaging = 0;
		g.heli_servo_averaging.save();
	}

	// mark swash as initialised
	heli_swash_initialised = true;
}

static void heli_move_servos_to_mid()
{
	heli_move_swash(0,0,500,0);
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
static void heli_move_swash(int roll_out, int pitch_out, int coll_out, int yaw_out)
{
    int yaw_offset = 0;

	if( g.heli_servo_manual == 1 ) {  // are we in manual servo mode? (i.e. swash set-up mode)?

	    // we must be in set-up mode so mark swash as uninitialised
	    heli_swash_initialised = false;

		// free up servo ranges
		if( g.heli_servo_1.get_reverse() ) {
			g.heli_servo_1.radio_min = 3000 - g.rc_3.radio_max + (g.heli_servo_1.radio_trim-1500);
			g.heli_servo_1.radio_max = 3000 - g.rc_3.radio_min + (g.heli_servo_1.radio_trim-1500);
		}else{
			g.heli_servo_1.radio_min = g.rc_3.radio_min + (g.heli_servo_1.radio_trim-1500);
			g.heli_servo_1.radio_max = g.rc_3.radio_max + (g.heli_servo_1.radio_trim-1500);
		}
		if( g.heli_servo_2.get_reverse() ) {
			g.heli_servo_2.radio_min = 3000 - g.rc_3.radio_max + (g.heli_servo_2.radio_trim-1500);
			g.heli_servo_2.radio_max = 3000 - g.rc_3.radio_min + (g.heli_servo_2.radio_trim-1500);
		}else{
			g.heli_servo_2.radio_min = g.rc_3.radio_min + (g.heli_servo_2.radio_trim-1500);
			g.heli_servo_2.radio_max = g.rc_3.radio_max + (g.heli_servo_2.radio_trim-1500);
		}
		if( g.heli_servo_3.get_reverse() ) {
			g.heli_servo_3.radio_min = 3000 - g.rc_3.radio_max + (g.heli_servo_3.radio_trim-1500);
			g.heli_servo_3.radio_max = 3000 - g.rc_3.radio_min + (g.heli_servo_3.radio_trim-1500);
		}else{
			g.heli_servo_3.radio_min = g.rc_3.radio_min + (g.heli_servo_3.radio_trim-1500);
			g.heli_servo_3.radio_max = g.rc_3.radio_max + (g.heli_servo_3.radio_trim-1500);
		}

	}else{  // regular flight mode

		// check if we need to reinitialise the swash
		if( !heli_swash_initialised ) {
			heli_init_swash();
		}

	    // ensure values are acceptable:
		roll_out = constrain(roll_out, (int)-g.heli_roll_max, (int)g.heli_roll_max);
		pitch_out = constrain(pitch_out, (int)-g.heli_pitch_max, (int)g.heli_pitch_max);
		coll_out = constrain(coll_out, 0, 1000);

		// rudder feed forward based on collective
		#if HIL_MODE == HIL_MODE_DISABLED  // don't do rudder feed forward in simulator
		if( !g.heli_ext_gyro_enabled ) {
			yaw_offset = g.heli_coll_yaw_effect * (coll_out - g.heli_coll_mid);
		}
		#endif
	}

	// swashplate servos
	g.heli_servo_1.servo_out = (heli_rollFactor[CH_1] * roll_out + heli_pitchFactor[CH_1] * pitch_out)/10 + coll_out + (g.heli_servo_1.radio_trim-1500);
	g.heli_servo_2.servo_out = (heli_rollFactor[CH_2] * roll_out + heli_pitchFactor[CH_2] * pitch_out)/10 + coll_out + (g.heli_servo_2.radio_trim-1500);
	g.heli_servo_3.servo_out = (heli_rollFactor[CH_3] * roll_out + heli_pitchFactor[CH_3] * pitch_out)/10 + coll_out + (g.heli_servo_3.radio_trim-1500);
	g.heli_servo_4.servo_out = yaw_out + yaw_offset;

	// use servo_out to calculate pwm_out and radio_out
	g.heli_servo_1.calc_pwm();
	g.heli_servo_2.calc_pwm();
	g.heli_servo_3.calc_pwm();
	g.heli_servo_4.calc_pwm();

	// add the servo values to the averaging
	heli_servo_out[0] += g.heli_servo_1.radio_out;
	heli_servo_out[1] += g.heli_servo_2.radio_out;
	heli_servo_out[2] += g.heli_servo_3.radio_out;
	heli_servo_out[3] += g.heli_servo_4.radio_out;
	heli_servo_out_count++;

	// is it time to move the servos?
	if( heli_servo_out_count >= g.heli_servo_averaging ) {

	    // average the values if necessary
	    if( g.heli_servo_averaging >= 2 ) {
		    heli_servo_out[0] /= g.heli_servo_averaging;
			heli_servo_out[1] /= g.heli_servo_averaging;
			heli_servo_out[2] /= g.heli_servo_averaging;
			heli_servo_out[3] /= g.heli_servo_averaging;
		}

		// actually move the servos
		_apm_rc->OutputCh(CH_1, heli_servo_out[0]);
		_apm_rc->OutputCh(CH_2, heli_servo_out[1]);
		_apm_rc->OutputCh(CH_3, heli_servo_out[2]);
		_apm_rc->OutputCh(CH_4, heli_servo_out[3]);

		// output gyro value
		if( g.heli_ext_gyro_enabled ) {
			_apm_rc->OutputCh(CH_7, g.heli_ext_gyro_gain);
		}

		#if INSTANT_PWM == 1
		// InstantPWM
		_apm_rc->Force_Out0_Out1();
		_apm_rc->Force_Out2_Out3();
		#endif

		// reset the averaging
		heli_servo_out_count = 0;
		heli_servo_out[0] = 0;
		heli_servo_out[1] = 0;
		heli_servo_out[2] = 0;
		heli_servo_out[3] = 0;
	}
}

void ArduCopter_Motors_Heli::init_out()
{
	#if INSTANT_PWM == 0
    _apm_rc->SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_3 | MSK_CH_4 );
	#endif
}

// these are not really motors, they're servos but we don't rename the function because it fits with the rest of the code better
void ArduCopter_Motors_Heli::output_armed()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if( g.heli_servo_manual == 1 ) {
		g.rc_1.servo_out = g.rc_1.control_in;
		g.rc_2.servo_out = g.rc_2.control_in;
		g.rc_3.servo_out = g.rc_3.control_in;
		g.rc_4.servo_out = g.rc_4.control_in;
	}

    //static int counter = 0;
	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	heli_move_swash( g.rc_1.servo_out, g.rc_2.servo_out, g.rc_3.servo_out, g.rc_4.servo_out );
}

// for helis - armed or disarmed we allow servos to move
void ArduCopter_Motors_Heli::output_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle, remove safety
		motor_auto_armed = true;
	}

	output_armed();
}

void ArduCopter_Motors_Heli::output_test()
{
}

// heli_angle_boost - adds a boost depending on roll/pitch values
// equivalent of quad's angle_boost function
// throttle value should be 0 ~ 1000
static int heli_get_angle_boost(int throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
	angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
	int throttle_above_mid = max(throttle - heli_throttle_mid,0);
	return throttle + throttle_above_mid*angle_boost_factor;

}

