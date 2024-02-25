#include "Arduino.h"
#include "Adafruit_LSM6DSOX.h"

Adafruit_LSM6DSOX::Adafruit_LSM6DSOX(void) {}

bool Adafruit_LSM6DSOX::_init(int32_t sensor_id){
    return true;
}

void Adafruit_LSM6DSOX::disableSPIMasterPullups(bool disable_pullups) {}

void Adafruit_LSM6DSOX::enableI2CMasterPullups(bool enable_pullups) {}
