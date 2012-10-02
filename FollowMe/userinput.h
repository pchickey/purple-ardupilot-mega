
#ifndef __FOLLOWME_USERINPUT_H__
#define __FOLLOWME_USERINPUT_H__

#include <AP_HAL.h>

class UserInput {
public:
    static void init(int side_btn_ch, int joy_x_ch, int joy_y_ch, int joy_btn_ch);
    static void print(AP_HAL::BetterStream* s);

private:
    static AP_HAL::AnalogSource* _joy_x;
    static AP_HAL::AnalogSource* _joy_y;
    static AP_HAL::DigitalSource* _side_btn;
    static AP_HAL::DigitalSource* _joy_btn;
};

#endif // __FOLLOWME_USERINPUT_H__

