/*
	APM_RC_APM1.cpp - Radio Control Library for Ardupilot Mega. Arduino
	Code by Jordi Mu�oz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	RC Input : PPM signal on IC4 pin
	RC Output : 11 Servo outputs (standard 20ms frame)

	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels

*/
#include "APM_RC_APM1.h"

#include <avr/interrupt.h>
#include "WProgram.h"

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint16_t APM_RC_APM1::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile uint8_t APM_RC_APM1::_radio_status=0;

/****************************************************
   Input Capture Interrupt ICP4 => PPM signal read
 ****************************************************/
void APM_RC_APM1::_timer4_capt_cb(void)
{
  static uint16_t ICR4_old;
  static uint8_t PPM_Counter=0;

  uint16_t Pulse;
  uint16_t Pulse_Width;

  Pulse=ICR4;
  if (Pulse<ICR4_old) {                       // Take care of the overflow of Timer4 (TOP=40000)
    Pulse_Width=(Pulse + 40000)-ICR4_old;     // Calculating pulse
  }
  else {
    Pulse_Width=Pulse-ICR4_old;               // Calculating pulse
  }

  if (Pulse_Width>8000) {                     // SYNC pulse?
    PPM_Counter=0;
  }
  else {
    if (PPM_Counter < NUM_CHANNELS) {         // Valid pulse channel?
	  _PWM_RAW[PPM_Counter++]=Pulse_Width;     // Saving pulse.

	  if (PPM_Counter >= NUM_CHANNELS) {
	    _radio_status = 1;
      }
    }
  }
  ICR4_old = Pulse;
}


// Constructors ////////////////////////////////////////////////////////////////

APM_RC_APM1::APM_RC_APM1()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_RC_APM1::Init( Arduino_Mega_ISR_Registry * isr_reg )
{

  isr_reg->register_signal(ISR_REGISTRY_TIMER4_CAPT, _timer4_capt_cb );

  // Init PWM Timer 1
  pinMode(11,OUTPUT); //OUT9 (PB5/OC1A)
  pinMode(12,OUTPUT); //OUT2 (PB6/OC1B)
  pinMode(13,OUTPUT); //OUT3 (PB7/OC1C)

  //Remember the registers not declared here remains zero by default...
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all...
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
  OCR1A = 0xFFFF; // Init ODR registers to nil output signal
  OCR1B = 0xFFFF;
  OCR1C = 0xFFFF;
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,

  // Init PWM Timer 3
  pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
  pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
  pinMode(5,OUTPUT); //OUT10(PE3/OC3A)
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  OCR3A = 0xFFFF; // Init ODR registers to nil output signal
  OCR3B = 0xFFFF;
  OCR3C = 0xFFFF;
  ICR3 = 40000; //50hz freq

  // Init PWM Timer 5
  pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
  pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
  pinMode(46,OUTPUT);  //OUT8 (PL3/OC5A)

  TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1));
  TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
  OCR5A = 0xFFFF; // Init ODR registers to nil output signal
  OCR5B = 0xFFFF;
  OCR5C = 0xFFFF;
  ICR5 = 40000; //50hz freq

  // Init PPM input and PWM Timer 4
  pinMode(49, INPUT);  // ICP4 pin (PL0) (PPM input)
  pinMode(7,OUTPUT);   //OUT5 (PH4/OC4B)
  pinMode(8,OUTPUT);   //OUT4 (PH5/OC4C)

  TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));
  //Prescaler set to 8, that give us a resolution of 0.5us
  // Input Capture rising edge
  TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4));
  OCR4B = 0xFFFF; // Init OCR registers to nil output signal
  OCR4C = 0xFFFF;
  OCR4A = 40000; ///50hz freq.

  //TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge).
  //TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
  TIMSK4 |= (1<<ICIE4); // Enable Input Capture interrupt. Timer interrupt mask
}

void APM_RC_APM1::OutputCh(uint8_t ch, uint16_t pwm)
{
  pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  pwm<<=1;   // pwm*2;

 switch(ch)
  {
    case 0:  OCR5B=pwm; break;  //ch0
    case 1:  OCR5C=pwm; break;  //ch1
    case 2:  OCR1B=pwm; break;  //ch2
    case 3:  OCR1C=pwm; break;  //ch3
    case 4:  OCR4C=pwm; break;  //ch4
    case 5:  OCR4B=pwm; break;  //ch5
    case 6:  OCR3C=pwm; break;  //ch6
    case 7:  OCR3B=pwm; break;  //ch7
    case 8:  OCR5A=pwm; break;  //ch8,  PL3
    case 9:  OCR1A=pwm; break;  //ch9,  PB5
    case 10: OCR3A=pwm; break;  //ch10, PE3
  }
}

uint16_t APM_RC_APM1::InputCh(uint8_t ch)
{
  uint16_t result;

  if (_HIL_override[ch] != 0) {
    return _HIL_override[ch];
  }

  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
  result =  _PWM_RAW[ch];
  if (result != _PWM_RAW[ch]) {
    result = _PWM_RAW[ch];   // if the results are different we make a third reading (this should be fine)
  }
  result >>= 1;  // Because timer runs at 0.5us we need to do value/2

  // Limit values to a valid range
  result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  _radio_status=0; // Radio channel read
  return(result);
}

uint8_t APM_RC_APM1::GetState(void)
{
  return(_radio_status);
}


// InstantPWM implementation
void APM_RC_APM1::Force_Out(void)
{
    Force_Out0_Out1();
    Force_Out2_Out3();
    Force_Out6_Out7();
}
// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer5). For quadcopters use
void APM_RC_APM1::Force_Out0_Out1(void)
{
  if (TCNT5>5000)  // We take care that there are not a pulse in the output
    TCNT5=39990;   // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer1). For quadcopters use
void APM_RC_APM1::Force_Out2_Out3(void)
{
  if (TCNT1>5000)
    TCNT1=39990;
}
// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer3). For quadcopters use
void APM_RC_APM1::Force_Out6_Out7(void)
{
  if (TCNT3>5000)
    TCNT3=39990;
}

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_APM1::setHIL(int16_t v[NUM_CHANNELS])
{
	uint8_t sum = 0;
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
	_radio_status = 1;
	if (sum == 0) {
		return 0;
	} else {
		return 1;
	}
}

void APM_RC_APM1::clearOverride(void)
{
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}


#endif // defined(ATMega1280)
