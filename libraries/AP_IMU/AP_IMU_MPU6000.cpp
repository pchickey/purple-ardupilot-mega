
#include "AP_IMU_MPU6000.h"

AP_IMU_MPU6000::AP_IMU_MPU6000( AP_Var::Key key ) :
	    _sensor_cal(key, PSTR("IMU_SENSOR_CAL"))
{ }

void AP_IMU_MPU6000::init( Start_style style,
                           void (*delay_cb)(unsigned long),
                           AP_PeriodicProcess * scheduler )
{
    scheduler->register_process( &AP_IMU_MPU6000::read );
    if (style == WARM_START) {
        _sensor_cal.load();
    } else {
        init_gyro(delay_cb);
        init_accel(delay_cb);
        _sensor_cal.save();
    }
}

void AP_IMU_MPU6000::init_accel( void (*delay)(unsigned long) )
{
    /* TODO */
    return;
}

void AP_IMU_MPU6000::init_gyro( void (*delay)(unsigned long) )
{
    /* TODO */
    return;
}

void AP_IMU_MPU6000::save(void)
{
    /* TODO */
    return;
}

bool AP_IMU_MPU6000::update( void )
{
    /* _gyro <== _data[3,4,5] */
    /* _accel <== _data[3,4,5] */
    /* _accel_filtered  <== filter ( _data[3,4,5], _accel_filtered ) */
    _sample_time = 200000;
    return true;
}

void AP_IMU_MPU6000::read()
{
    /* SPI transactions to _data */
}

