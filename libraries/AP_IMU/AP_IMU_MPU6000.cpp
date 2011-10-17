
#include "AP_IMU_MPU6000.h"

#include <wiring.h>
#include <SPI.h>

// MPU 6000 registers
#define MPUREG_WHOAMI 0x75 //
#define MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38 
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 // 
#define MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define MPUREG_PWR_MGMT_1 0x6B //
#define MPUREG_PWR_MGMT_2 0x6C //
  
// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      0x10
#define BIT_RAW_RDY_EN        0x01
#define BIT_I2C_IF_DIS              0x10

uint16_t AP_IMU_MPU6000::_data[7];
int      AP_IMU_MPU6000::_cs_pin;

AP_IMU_MPU6000::AP_IMU_MPU6000( AP_Var::Key key, int cs_pin ) :
	    _sensor_cal(key, PSTR("IMU_SENSOR_CAL"))
{ 
  _cs_pin = cs_pin; /* can't use initializer list,  is static */
}

void AP_IMU_MPU6000::init( Start_style style,
                           void (*delay_cb)(unsigned long),
                           AP_PeriodicProcess * scheduler )
{
    scheduler->register_process( &AP_IMU_MPU6000::read );
    hardware_init();
    
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
    int byte_H;
    int byte_L;
  
  // Read AccelX
    byte_H = register_read(MPUREG_ACCEL_XOUT_H);
    byte_L = register_read(MPUREG_ACCEL_XOUT_L);
    _data[0] = (byte_H<<8)| byte_L;
    // Read AccelY
    byte_H = register_read(MPUREG_ACCEL_YOUT_H);
    byte_L = register_read(MPUREG_ACCEL_YOUT_L);
    _data[1] = (byte_H<<8)| byte_L;
    // Read AccelZ
    byte_H = register_read(MPUREG_ACCEL_ZOUT_H);
    byte_L = register_read(MPUREG_ACCEL_ZOUT_L);
    _data[2] = (byte_H<<8)| byte_L;

    // Read GyroX
    byte_H = register_read(MPUREG_GYRO_XOUT_H);
    byte_L = register_read(MPUREG_GYRO_XOUT_L);
    _data[3] = (byte_H<<8)| byte_L;
    // Read GyroY
    byte_H = register_read(MPUREG_GYRO_YOUT_H);
    byte_L = register_read(MPUREG_GYRO_YOUT_L);
    _data[4] = (byte_H<<8)| byte_L;
    // Read GyroZ
    byte_H = register_read(MPUREG_GYRO_ZOUT_H);
    byte_L = register_read(MPUREG_GYRO_ZOUT_L);
    _data[5] = (byte_H<<8)| byte_L;

    // Read Temp
    byte_H = register_read(MPUREG_TEMP_OUT_H);
    byte_L = register_read(MPUREG_TEMP_OUT_L);
    _data[6] = (byte_H<<8)| byte_L; 


}

uint8_t AP_IMU_MPU6000::register_read( uint8_t reg )
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg | 0x80; // Set most significant bit

  digitalWrite(_cs_pin, LOW);

  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);

  digitalWrite(_cs_pin, HIGH);

  return return_value;
}

void AP_IMU_MPU6000::register_write(uint8_t reg, uint8_t val)
{
  uint8_t dump;
  digitalWrite(_cs_pin, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(val);
  digitalWrite(_cs_pin, HIGH);
}

void AP_IMU_MPU6000::hardware_init()
{
    // Need to initialize SPI if it hasn't already.
    SPI.begin();
    #if F_CPU == 16000000
    SPI.setClockDivider(SPI_CLOCK_DIV16); // SPI at 1MHz
    #else
    # error MPU6000 requires SPI at 1MHZ! Need appropriate SPI clock divider.
    #endif

    // MPU6000 chip select setup
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    delay(1);
    
    // Chip reset
    register_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    // Wake up device and select GyroZ clock (better performance)
    register_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delay(1);
    // Disable I2C bus (recommended on datasheet)
    register_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    delay(1);
    // SAMPLE RATE
    register_write(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
    delay(1);
    // FS & DLPF   FS=1000º/s, DLPF = 42Hz (low pass filter)
    register_write(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
    delay(1);
    register_write(MPUREG_GYRO_CONFIG,BITS_FS_500DPS);  // Gyro scale 500º/s
    delay(1);
    register_write(MPUREG_ACCEL_CONFIG,0x08);           // Accel scele 4g (4096LSB/g)
    delay(1);    
    // INT CFG => Interrupt on Data Ready
    register_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    delay(1);
    register_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    delay(1);
    // Oscillator set
    // register_write(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    delay(1);

}


