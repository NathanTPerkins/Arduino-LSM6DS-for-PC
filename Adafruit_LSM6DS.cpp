#include "Adafruit_LSM6DS.h"

static const float _data_rate_arr[] = {
    [LSM6DS_RATE_SHUTDOWN] = 0.0f,    [LSM6DS_RATE_12_5_HZ] = 12.5f,
    [LSM6DS_RATE_26_HZ] = 26.0f,      [LSM6DS_RATE_52_HZ] = 52.0f,
    [LSM6DS_RATE_104_HZ] = 104.0f,    [LSM6DS_RATE_208_HZ] = 208.0f,
    [LSM6DS_RATE_416_HZ] = 416.0f,    [LSM6DS_RATE_833_HZ] = 833.0f,
    [LSM6DS_RATE_1_66K_HZ] = 1660.0f, [LSM6DS_RATE_3_33K_HZ] = 3330.0f,
    [LSM6DS_RATE_6_66K_HZ] = 6660.0f,
};

Adafruit_LSM6DS::Adafruit_LSM6DS() {
    this->random_values = true;
    srand(time(NULL));    
    this->_running = false;
}

Adafruit_LSM6DS::~Adafruit_LSM6DS() {
    if(this->temp_sensor != NULL){
        delete this->temp_sensor;
    }
    if(this->accel_sensor != NULL){
        delete this->accel_sensor;
    }
    if(this->gyro_sensor != NULL){
        delete this->gyro_sensor;
    }
    #ifdef USING_CSV
    if(this->sensor_data != NULL){
        delete this->sensor_data;
    }
    #endif
}

bool Adafruit_LSM6DS::_init(int32_t sensor_id){
    setAccelDataRate(LSM6DS_RATE_104_HZ);
    setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

    setGyroDataRate(LSM6DS_RATE_104_HZ);
    setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);

    delay(10);

    this->_running = true;

    if(this->temp_sensor != NULL){
        delete this->temp_sensor;
    }
    if(this->accel_sensor != NULL){
        delete this->accel_sensor;
    }
    if(this->gyro_sensor != NULL){
        delete this->gyro_sensor;
    }

    temp_sensor = new Adafruit_LSM6DS_Temp(this);
    accel_sensor = new Adafruit_LSM6DS_Accelerometer(this);
    gyro_sensor = new Adafruit_LSM6DS_Gyro(this);

    return false;
}

uint8_t Adafruit_LSM6DS::chipID(){
    return 1;
}

bool Adafruit_LSM6DS::begin_I2C(uint8_t i2c_address, int32_t sensor_id){
    return _init(sensor_id);
}

bool Adafruit_LSM6DS::begin_SPI(uint8_t cs_pin, int32_t sensor_id, uint32_t frequency){
    return _init(sensor_id);
}

bool Adafruit_LSM6DS::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin, int8_t mosi_pin, int32_t sensor_id, uint32_t frequency){
    return _init(sensor_id);
}

void Adafruit_LSM6DS::reset(){}

Adafruit_Sensor* Adafruit_LSM6DS::getTemperatureSensor(){
    return this->temp_sensor;
}

Adafruit_Sensor* Adafruit_LSM6DS::getAccelerometerSensor(){
    return this->accel_sensor;
}

Adafruit_Sensor* Adafruit_LSM6DS::getGyroSensor(){
    return this->gyro_sensor;
}

bool Adafruit_LSM6DS::getEvent(sensors_event_t* acceleration_event, sensors_event_t* gyro_event, sensors_event_t* temp_event){
    uint32_t t = millis();
    _read();

    fillAccelEvent(acceleration_event, t);
    fillGyroEvent(gyro_event, t);
    fillTempEvent(temp_event, t);
    return true;
}

void Adafruit_LSM6DS::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {
    memset(temp, 0, sizeof(sensors_event_t));
    temp->version = sizeof(sensors_event_t);
    temp->sensor_id = _sensorid_temp;
    temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    temp->timestamp = timestamp;
    temp->temperature = temperature;
}

void Adafruit_LSM6DS::fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp) {
    memset(gyro, 0, sizeof(sensors_event_t));
    gyro->version = 1;
    gyro->sensor_id = _sensorid_gyro;
    gyro->type = SENSOR_TYPE_GYROSCOPE;
    gyro->timestamp = timestamp;
    gyro->gyro.x = gyroX;
    gyro->gyro.y = gyroY;
    gyro->gyro.z = gyroZ;
}

void Adafruit_LSM6DS::fillAccelEvent(sensors_event_t *accel,
                                     uint32_t timestamp) {
    memset(accel, 0, sizeof(sensors_event_t));
    accel->version = 1;
    accel->sensor_id = _sensorid_accel;
    accel->type = SENSOR_TYPE_ACCELEROMETER;
    accel->timestamp = timestamp;
    accel->acceleration.x = accX;
    accel->acceleration.y = accY;
    accel->acceleration.z = accZ;
}

lsm6ds_data_rate_t Adafruit_LSM6DS::getAccelDataRate(void) {
    return this->_accel_data_rate;
}

void Adafruit_LSM6DS::setAccelDataRate(lsm6ds_data_rate_t data_rate) {
    this->_accel_data_rate = data_rate;
}

lsm6ds_accel_range_t Adafruit_LSM6DS::getAccelRange(void) {
    return this->accelRangeBuffered;
}

void Adafruit_LSM6DS::setAccelRange(lsm6ds_accel_range_t new_range) {
    this->accelRangeBuffered = new_range;
}

lsm6ds_data_rate_t Adafruit_LSM6DS::getGyroDataRate(void) {
    return this->_gyro_data_rate;
}

void Adafruit_LSM6DS::setGyroDataRate(lsm6ds_data_rate_t data_rate) {
    this->_gyro_data_rate = data_rate;
}

lsm6ds_gyro_range_t Adafruit_LSM6DS::getGyroRange(void) {
    return gyroRangeBuffered;
}

void Adafruit_LSM6DS::setGyroRange(lsm6ds_gyro_range_t new_range) {
    this->gyroRangeBuffered = new_range;
}

void Adafruit_LSM6DS::highPassFilter(bool filter_enabled,
                                     lsm6ds_hp_filter_t filter) {
}

void Adafruit_LSM6DS::_read(void) {
    this->accX = rand() / rand();
    this->accY = rand() / rand();
    this->accZ = rand() / rand();

    this->gyroX = rand() / rand();
    this->gyroY = rand() / rand();
    this->gyroZ = rand() / rand();

    this->temperature = rand() / rand();
}

void Adafruit_LSM6DS::configIntOutputs(bool active_low, bool open_drain) {
}

void Adafruit_LSM6DS::configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
                                 bool step_detect, bool wakeup) {
}

void Adafruit_LSM6DS::configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl) {
}

void Adafruit_LSM6DS_Gyro::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "LSM6DS_G", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->min_value = -34.91; /* -2000 dps -> rad/s (radians per second) */
    sensor->max_value = +34.91;
    sensor->resolution = 7.6358e-5; /* 4.375 mdps -> rad/s */
}

bool Adafruit_LSM6DS_Gyro::getEvent(sensors_event_t *event) {
    _theLSM6DS->_read();
    _theLSM6DS->fillGyroEvent(event, millis());

    return true;
}

void Adafruit_LSM6DS_Accelerometer::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "LSM6DS_A", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->min_value = -156.9064F; /*  -16g = 156.9064 m/s^2  */
    sensor->max_value = 156.9064F;  /* 16g = 156.9064 m/s^2  */
    sensor->resolution = 0.061;     /* 0.061 mg/LSB at +-2g */
}

bool Adafruit_LSM6DS_Accelerometer::getEvent(sensors_event_t *event) {
    _theLSM6DS->_read();
    _theLSM6DS->fillAccelEvent(event, millis());

    return true;
}

void Adafruit_LSM6DS_Temp::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "LSM6DS_T", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->min_value = -40;
    sensor->max_value = 85;
    sensor->resolution = 1; /* not a great sensor */
}

bool Adafruit_LSM6DS_Temp::getEvent(sensors_event_t *event) {
    _theLSM6DS->_read();
    _theLSM6DS->fillTempEvent(event, millis());

    return true;
}

void Adafruit_LSM6DS::enablePedometer(bool enable) {
}

void Adafruit_LSM6DS::enableWakeup(bool enable, uint8_t duration,
                                   uint8_t thresh) {
}

bool Adafruit_LSM6DS::awake(void) {
    return true;
}

bool Adafruit_LSM6DS::shake(void) {
    return true;
}

void Adafruit_LSM6DS::resetPedometer(void) {
}

uint16_t Adafruit_LSM6DS::readPedometer(void) {
    return 1;
}

float Adafruit_LSM6DS::accelerationSampleRate(void) {
    return _data_rate_arr[this->getAccelDataRate()];
}

int Adafruit_LSM6DS::accelerationAvailable(void) {
    return 1;
}

int Adafruit_LSM6DS::readAcceleration(float &x, float &y, float &z) {
    int16_t data[3];

    data[0] = rand() / rand();
    data[1] = rand() / rand();
    data[2] = rand() / rand();

    // scale to range of -4 – 4
    x = data[0] * 4.0 / 32768.0;
    y = data[1] * 4.0 / 32768.0;
    z = data[2] * 4.0 / 32768.0;

    return 1;
}

float Adafruit_LSM6DS::gyroscopeSampleRate(void) {
    return _data_rate_arr[this->getGyroDataRate()];
}

int Adafruit_LSM6DS::gyroscopeAvailable(void) {
    return 1;
}

int Adafruit_LSM6DS::readGyroscope(float &x, float &y, float &z) {
    int16_t data[3];

    data[0] = rand() / rand();
    data[1] = rand() / rand();
    data[2] = rand() / rand();

    // scale to range of -2000 – 2000
    x = data[0] * 2000.0 / 32768.0;
    y = data[1] * 2000.0 / 32768.0;
    z = data[2] * 2000.0 / 32768.0;

    return 1;
}
