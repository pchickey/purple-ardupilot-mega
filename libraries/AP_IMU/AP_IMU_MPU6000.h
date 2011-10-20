
#ifndef __AP_IMU_MPU6000_H__
#define __AP_IMU_MPU6000_H__

#include <stdint.h>

#include "IMU.h"

#include "../AP_Common/AP_Common.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"

class AP_IMU_MPU6000 : public IMU
{
public:
        AP_IMU_MPU6000( AP_Var::Key, int cs_pin );
        virtual void init( Start_style style,
                          void (*delay_cb) (unsigned long),
                          AP_PeriodicProcess *scheduler );
        virtual bool update(void);
        virtual void init_accel(void (*delay_cb)(unsigned long t) = delay);
        virtual float gx(void) { return _sensor_cal[0]; }
        virtual float gy(void) { return _sensor_cal[1]; }
        virtual float gz(void) { return _sensor_cal[2]; }
        virtual float ax(void) { return _sensor_cal[3]; }
        virtual float ay(void) { return _sensor_cal[4]; }
        virtual float az(void) { return _sensor_cal[5]; }
        virtual void  ax(const float v) { _sensor_cal[3] = v; }
        virtual void  ay(const float v) { _sensor_cal[4] = v; }
        virtual void  az(const float v) { _sensor_cal[5] = v; }

        virtual void init_gyro(void (*delay_cb)(unsigned long t) = delay);
        virtual void save(void);
        /* AP_IMU_MPU6000 is de-facto a singleton. We use ::read as a
         * static method to simplify registering it as a callback.
         */
        static void read(void);
private:
        static uint8_t register_read(uint8_t reg);
        static void    register_write(uint8_t reg, uint8_t val);

        AP_VarA<uint32_t,6>     _sensor_cal;
        /* ::read uses these bits of data, so they need to be static. */
        static uint16_t         _data[7];
        static int              _cs_pin;

        void hardware_init();

        static const float _gyro_gain;
        static const float _accel_gain;
};

#endif // __AP_IMU_MPU6000_H__

