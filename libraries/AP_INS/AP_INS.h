
#ifndef __AP_INS_H__
#define __AP_INS_H__

/* AP_INS is an abstraction for INertial Sensor data (gyro and accel)
 * which is correctly aligned to the body axes, and scaled to SI units.
 */
class AP_INS 
{
  public:
  AP_INS() {}

  /* Update the sensor data, so that getters are nonblocking. 
   * Returns a bool of whether data was updated or not.
   */
  virtual bool update() = 0;

  /* Getters for individual gyro axes. 
   * Gyros have correct coordinate frame and units (degrees per second).
   */
  virtual float gx() = 0;
  virtual float gy() = 0;
  virtual float gz() = 0;

  /* Same data as above gyro getters, written to array as { gx, gy, gz } */ 
  virtual void get_gyros( float * ) = 0;

  /* Getters for individual accel axes.
   * Accels have correct coordinate frame ( flat level ax, ay = 0; az = -9.81)
   * and units (meters per second squared).
   */
  virtual float ax() = 0;
  virtual float ay() = 0;
  virtual float az() = 0;
 
  /* Same data as above accel getters, written to array as { ax, ay, az } */
  virtual void get_accels( float * ) = 0;
 
  /* Same data as above accel and gyro getters, written to array as 
   * { gx, gy, gz, ax, ay, az }
   */
  virtual void get_sensors( float * ) = 0;
  
  /* Temperature, in degrees celsius, of the gyro. */
  virtual float temperature() = 0;

  virtual uint32_t sample_time() = 0;
};

#include "AP_INS_Oilpan.h"

#endif // __AP_INS_H__

