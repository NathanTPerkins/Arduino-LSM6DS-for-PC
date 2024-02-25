#ifndef ADAFRUIT_LSM6DS_H
#define ADAFRUIT_LSM6DS_H

#include "Arduino.h"
// #include <Adafruit_BusIO_Register.h>
// #include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
// #include <Wire.h>

#include <limits.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef USING_CSV
#include <CSVParser.h>
#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2
#define GYRO_X 3
#define GYRO_Y 4
#define GYRO_Z 5
#define TEMP 6
#endif

/*
acc_x,
acc_y,
acc_z,
gyro_x,
gyro_y,
gyro_z,
temp,
*/

#define LSM6DS_I2CADDR_DEFAULT 0x6A ///< LSM6DS default i2c address

#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0x0F         ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define LSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define LSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define LSM6DS_STATUS_REG 0X1E     ///< Status register
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define LSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define LSM6DS_TAP_CFG 0x58        ///< Tap/pedometer configuration
#define LSM6DS_WAKEUP_THS                                                      \
  0x5B ///< Single and double-tap function threshold register
#define LSM6DS_WAKEUP_DUR                                                      \
  0x5C ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DS_MD1_CFG 0x5E ///< Functions routing on INT1 register

/** The accelerometer data rate */
typedef enum data_rate {
  LSM6DS_RATE_SHUTDOWN,
  LSM6DS_RATE_12_5_HZ,
  LSM6DS_RATE_26_HZ,
  LSM6DS_RATE_52_HZ,
  LSM6DS_RATE_104_HZ,
  LSM6DS_RATE_208_HZ,
  LSM6DS_RATE_416_HZ,
  LSM6DS_RATE_833_HZ,
  LSM6DS_RATE_1_66K_HZ,
  LSM6DS_RATE_3_33K_HZ,
  LSM6DS_RATE_6_66K_HZ,
} lsm6ds_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DS_ACCEL_RANGE_2_G,
  LSM6DS_ACCEL_RANGE_16_G,
  LSM6DS_ACCEL_RANGE_4_G,
  LSM6DS_ACCEL_RANGE_8_G
} lsm6ds_accel_range_t;

/** The gyro data range */
typedef enum gyro_range {
  LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
  LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
  LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
  LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
  LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
  ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6ds_gyro_range_t;

/** The high pass filter bandwidth */
typedef enum hpf_range {
  LSM6DS_HPF_ODR_DIV_50 = 0,
  LSM6DS_HPF_ODR_DIV_100 = 1,
  LSM6DS_HPF_ODR_DIV_9 = 2,
  LSM6DS_HPF_ODR_DIV_400 = 3,
} lsm6ds_hp_filter_t;

class Adafruit_LSM6DS;

class Adafruit_LSM6DS_Temp : public Adafruit_Sensor {
public:

  Adafruit_LSM6DS_Temp(Adafruit_LSM6DS* _parent) {_theLSM6DS = _parent;}
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:

  int _sensorID = 0x6D0;
  Adafruit_LSM6DS* _theLSM6DS = NULL;

};

class Adafruit_LSM6DS_Accelerometer : public Adafruit_Sensor {
public:

  Adafruit_LSM6DS_Accelerometer(Adafruit_LSM6DS* _parent) {_theLSM6DS = _parent;}
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x6D1;
  Adafruit_LSM6DS* _theLSM6DS = NULL;

};

class Adafruit_LSM6DS_Gyro : public Adafruit_Sensor {
public:

  Adafruit_LSM6DS_Gyro(Adafruit_LSM6DS* _parent) {_theLSM6DS = _parent;}
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x6D2;
  Adafruit_LSM6DS* _theLSM6DS = NULL;

};

class Adafruit_LSM6DS {
public:

  Adafruit_LSM6DS();
  virtual ~Adafruit_LSM6DS();

  bool begin_I2C(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT, int32_t sensorID = 0);

  bool begin_SPI(uint8_t, int32_t sensorID = 0, uint32_t frequency = 1000000);

  bool begin_SPI(int8_t, int8_t, int8_t, int8_t, int32_t sensorID = 0, uint32_t frequency = 1000000);

  bool getEvent(sensors_event_t*, sensors_event_t*, sensors_event_t*);

  lsm6ds_data_rate_t getAccelDataRate();
  void setAccelDataRate(lsm6ds_data_rate_t);

  lsm6ds_accel_range_t getAccelRange();
  void setAccelRange(lsm6ds_accel_range_t);

  lsm6ds_data_rate_t getGyroDataRate();
  void setGyroDataRate(lsm6ds_data_rate_t);

  lsm6ds_gyro_range_t getGyroRange();
  void setGyroRange(lsm6ds_gyro_range_t);

  void reset();
  void configIntOutputs(bool, bool);
  void configInt1(bool, bool, bool, bool step_detect = false, bool wakeup = false);
  void configInt2(bool, bool, bool);
  void highPassFilter(bool, lsm6ds_hp_filter_t);

  void enableWakeup(bool, uint8_t duration = 0, uint8_t thresh = 20);
  bool awake();
  bool shake();

  void enablePedometer(bool);
  void resetPedometer();
  uint16_t readPedometer();
  
  int readAcceleration(float&, float&, float&);
  float accelerationSampleRate();
  int accelerationAvailable();

  int readGyroscope(float&, float&, float&);
  float gyroscopeSampleRate();
  int gyroscopeAvailable();

  int16_t rawAccX,
    rawAccY,
    rawAccZ,
    rawTemp,
    rawGyroX,
    rawGyroY,
    rawGyroZ;

  float temperature,
    accX,
    accY,
    accZ,
    gyroX,
    gyroY,
    gyroZ;

  Adafruit_Sensor* getTemperatureSensor();
  Adafruit_Sensor* getAccelerometerSensor();
  Adafruit_Sensor* getGyroSensor();

protected:
  uint8_t chipID();
  uint8_t status();
  virtual void _read();
  virtual bool _init(int32_t);

  uint16_t _sensorid_accel,
    _sensorid_gyro,
    _sensorid_temp;

  float temperature_sensitivity = 256.0F;

  lsm6ds_data_rate_t _accel_data_rate = LSM6DS_RATE_104_HZ;
  lsm6ds_data_rate_t _gyro_data_rate = LSM6DS_RATE_104_HZ;

  Adafruit_LSM6DS_Temp* temp_sensor = NULL;
  Adafruit_LSM6DS_Accelerometer* accel_sensor = NULL;
  Adafruit_LSM6DS_Gyro* gyro_sensor = NULL;

  lsm6ds_accel_range_t accelRangeBuffered = LSM6DS_ACCEL_RANGE_2_G;
  lsm6ds_gyro_range_t gyroRangeBuffered = LSM6DS_GYRO_RANGE_250_DPS;

  bool _running;
  bool random_values;
  char *_filename;
  int file_index;

  #ifdef USING_CSV
  csv_parser::parser* sensor_data;
  #endif

private:

  friend class Adafruit_LSM6DS_Temp;

  friend class Adafruit_LSM6DS_Accelerometer;

  friend class Adafruit_LSM6DS_Gyro;

  void fillTempEvent(sensors_event_t*, uint32_t);
  void fillAccelEvent(sensors_event_t*, uint32_t);
  void fillGyroEvent(sensors_event_t*, uint32_t);
};

#endif