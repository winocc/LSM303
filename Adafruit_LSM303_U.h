/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __LSM303_H__
#define __LSM303_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define LSM303_ADDRESS_ACCEL          (0x3A >> 1)         // 0011101x
    #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                                     // DEFAULT    TYPE
      LSM303_REGISTER_ACCEL_WHO_AM_I_M          = 0x0F,
      LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
      LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
      LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
      LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
      LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
      LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
      LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
      LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
      LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
      LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
      LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
      LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
      LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
      LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
      LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
      LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
      LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
      LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
      LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
      LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
      LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
      LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
      LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
      LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
      LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
    } lsm303AccelRegisters_t;
    
    typedef enum
    {
      LSM303_REGISTER_MAG_WHO_AM_I_M          = 0x0F,
      LSM303_REGISTER_MAG_CTRL_REG1_M         = 0x20,
      LSM303_REGISTER_MAG_CTRL_REG2_M         = 0x21,
      LSM303_REGISTER_MAG_CTRL_REG3_M         = 0x22,
      LSM303_REGISTER_MAG_CTRL_REG4_M         = 0x23,
      LSM303_REGISTER_MAG_CTRL_REG5_M         = 0x24,
      LSM303_REGISTER_MAG_STATUS_REG_M        = 0x27,
      LSM303_REGISTER_MAG_OUT_X_L_M           = 0x28,
      LSM303_REGISTER_MAG_OUT_X_H_M           = 0x29,
      LSM303_REGISTER_MAG_OUT_Z_L_M           = 0x2A,
      LSM303_REGISTER_MAG_OUT_Z_H_M           = 0x2B,
      LSM303_REGISTER_MAG_OUT_Y_L_M           = 0x2C,
      LSM303_REGISTER_MAG_OUT_Y_H_M           = 0x2D,
      LSM303_REGISTER_MAG_TEMP_L_M            = 0x2E,
      LSM303_REGISTER_MAG_TEMP_H_M            = 0x2F,
      LSM303_REGISTER_MAG_INT_CFG_M           = 0x30,
      LSM303_REGISTER_MAG_INT_SRC_M           = 0x31,
      LSM303_REGISTER_MAG_INT_THS_L_M         = 0x32,
      LSM303_REGISTER_MAG_INT_THS_H_M         = 0x33
    } lsm303MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } lsm303MagGain;	
/*=========================================================================*/

/*=========================================================================
    ACCELEROMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_ACCELRATE_POWERDOWN                = 0x0,  // Power-down mode
      LSM303_ACCELRATE_10HZ                     = 0x1,  // 10Hz
      LSM303_ACCELRATE_50HZ                     = 0x2,  // 50Hz
      LSM303_ACCELRATE_100HZ                    = 0x3,  // 100Hz
      LSM303_ACCELRATE_200HZ                    = 0x4,  // 200Hz
      LSM303_ACCELRATE_400HZ                    = 0x5,  // 400Hz
      LSM303_ACCELRATE_800HZ                    = 0x6,  // 800Hz
      LSM303_ACCELRATE_NA                       = 0x7,  // NA
    } lsm303AccelRate;	
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
      LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
      LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
      LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
      LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
      LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
      LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
      LSM303_MAGRATE_220                        = 0x07   // 200 Hz
    } lsm303MagRate;	
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct lsm303MagData_s
    {
        float x;
        float y;
        float z;
    } lsm303MagData;
/*=========================================================================*/

/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct lsm303AccelData_s
    {
      float x;
      float y;
      float z;
    } lsm303AccelData;
/*=========================================================================*/
	
/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
    #define LSM303_ACCEL_ID               0x41
    #define LSM303_MAG_ID                 0x3D
/*=========================================================================*/

/* Unified sensor driver for the accelerometer */
class Adafruit_LSM303_Accel_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_LSM303_Accel_Unified(int32_t sensorID = -1);
  
    bool begin(void);
    bool setOdr(lsm303AccelRate odr);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

  private:
    lsm303AccelData _accelData;   // Last read accelerometer data will be available here
    int32_t         _sensorID;
    
    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);
};

/* Unified sensor driver for the magnetometer */
class Adafruit_LSM303_Mag_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_LSM303_Mag_Unified(int32_t sensorID = -1);
  
    bool begin(void);
    void enableAutoRange(bool enable);
    void setMagGain(lsm303MagGain gain);
    void setMagRate(lsm303MagRate rate);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

  private:
    lsm303MagGain   _magGain;
    lsm303MagData   _magData;     // Last read magnetometer data will be available here
    int32_t         _sensorID;
    bool            _autoRangeEnabled;
    
    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);
};

#endif
