#ifndef ADAFRUIT_LSM6DOS032_H
#define ADAFRUIT_LSM6DOS032_H

#include "Adafruit_LSM6DSOX.h"

#define LSM6DSO32_CHIP_ID 0x6C ///< LSM6DSO32 default device id from WHOAMI

/** The accelerometer data range */
typedef enum dso32_accel_range {
    LSM6DSO32_ACCEL_RANGE_4_G,
    LSM6DSO32_ACCEL_RANGE_32_G,
    LSM6DSO32_ACCEL_RANGE_8_G,
    LSM6DSO32_ACCEL_RANGE_16_G
} lsm6dso32_accel_range_t;

class Adafruit_LSM6DSO32 : public Adafruit_LSM6DSOX {
public:
    Adafruit_LSM6DSO32(const char * filename = NULL);

    lsm6dso32_accel_range_t getAccelRange(void);
    void setAccelRange(lsm6dso32_accel_range_t new_range);
    void _read(void);

    void show_simulated_file();
    void increase_file_index();


private:

    lsm6dso32_accel_range_t _range = LSM6DSO32_ACCEL_RANGE_4_G;

    bool _init(int32_t sensor_id);
};

#endif