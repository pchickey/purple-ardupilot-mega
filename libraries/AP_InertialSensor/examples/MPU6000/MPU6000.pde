// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor MPU6000 driver.
//

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_InertialSensor_MPU6000 ins;

void setup(void)
{
    hal.console->println("AP_InertialSensor_MPU6000 startup...");

    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);

    ins.init();
    hal.console->println("Complete. Reading:");
}

void loop(void)
{
    float accel[3];
    float gyro[3];
    float temperature;

    hal.scheduler->delay(20);
    ins.update();
    ins.get_gyros(gyro);
    ins.get_accels(accel);
    temperature = ins.temperature();

    hal.console->printf_P(
            PSTR("AX: %f  AY: %f  AZ: %f  GX: %f  GY: %f  GZ: %f T=%f\n"),
            accel[0], accel[1], accel[2], 
            gyro[0], gyro[1], gyro[2], temperature);
}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
