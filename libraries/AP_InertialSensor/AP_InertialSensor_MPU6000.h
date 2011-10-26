
#ifndef __AP_INERTIAL_SENSOR_MPU6000_H__
#define __AP_INERTIAL_SENSOR_MPU6000_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

class AP_InertialSensor_MPU6000 : public AP_InertialSensor
{
  public:

  AP_InertialSensor_MPU6000( int cs_pin );

  void init( AP_PeriodicProcess * scheduler );

  /* Concrete implementation of AP_InertialSensor functions: */
  bool update();
  float gx();
  float gy();
  float gz();
  void get_gyros( float * );
  float ax();
  float ay();
  float az();
  void get_accels( float * );
  void get_sensors( float * );
  float temperature();
  uint32_t sample_time();

  static void read();
  static uint8_t register_read( uint8_t reg );
  static void register_write( uint8_t reg, uint8_t val );
  static void hardware_init();

  private:

  Vector3f _gyro;
  Vector3f _accel;
  float _temp;
  
  float _temp_to_celsius( uint16_t );

  static const float _accel_scale; 
  static const float _gyro_scale;

  static int _cs_pin;
  static uint16_t _data[7];
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__

