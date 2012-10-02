
#include <AP_HAL.h>
#include "userinput.h"

extern const AP_HAL::HAL& hal;

AP_HAL::AnalogSource* UserInput::_joy_x;
AP_HAL::AnalogSource* UserInput::_joy_y;
AP_HAL::DigitalSource* UserInput::_side_btn;
AP_HAL::DigitalSource* UserInput::_joy_btn;

void UserInput::init( int side_btn_ch, int joy_x_ch,
                      int joy_y_ch, int joy_btn_ch) {

  _joy_x = hal.analogin->channel(joy_x_ch);
  _joy_y = hal.analogin->channel(joy_y_ch);
  _side_btn = hal.gpio->channel(side_btn_ch);
  _joy_btn = hal.gpio->channel(joy_btn_ch);
}

void UserInput::print(AP_HAL::BetterStream* s) {
  s->printf_P(PSTR("side: %d joy: %f, %f, %d\r\n"),
      (int) _side_btn->read(), 
      _joy_x->read(),
      _joy_y->read(),
      (int) _joy_btn->read());
}

