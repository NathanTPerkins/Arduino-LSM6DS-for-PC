#include "Adafruit_LSM6DSO32.h"

#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_LSM6DSOX.h"

Adafruit_LSM6DSO32::Adafruit_LSM6DSO32(const char *filename) {
    if(filename == NULL){
        this->random_values = true;
        srand(time(NULL));
        return;
    }
    this->random_values = false;
    
    #ifdef USING_CSV
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    this->sensor_data = new csv_parser::parser(filename, 10);
    this->file_index = 0;
    #else
    srand(time(NULL));
    #endif
    
    this->_running = false;
}

bool Adafruit_LSM6DSO32::_init(int32_t sensor_id) {
  Adafruit_LSM6DS::_init(sensor_id);
  return true;
}

void Adafruit_LSM6DSO32::_read(void) {
  if(this->random_values){
    this->accX = rand() / rand();
    this->accY = rand() / rand();
    this->accZ = rand() / rand();

    this->gyroX = rand() / rand();
    this->gyroY = rand() / rand();
    this->gyroZ = rand() / rand();

    this->temperature = rand() / rand();
  }

  #ifdef USING_CSV

  if(this->file_index >= this->sensor_data->getSize()){
    this->accX = 0;
    this->accY = 0;
    this->accZ = 0;

    this->gyroX = 0;
    this->gyroY = 0;
    this->gyroZ = 0;

    this->temperature = 0;
    return;
  }

  this->accX = atof(this->sensor_data->operator[](this->file_index)[ACC_X]);
  this->accY = atof(this->sensor_data->operator[](this->file_index)[ACC_Y]);
  this->accZ = atof(this->sensor_data->operator[](this->file_index)[ACC_Z]);

  this->gyroX = atof(this->sensor_data->operator[](this->file_index)[GYRO_X]);
  this->gyroY = atof(this->sensor_data->operator[](this->file_index)[GYRO_Y]);
  this->gyroZ = atof(this->sensor_data->operator[](this->file_index)[GYRO_Z]);

  this->temperature = atof(this->sensor_data->operator[](this->file_index)[TEMP]);
  #else
  this->accX = rand() / rand();
  this->accY = rand() / rand();
  this->accZ = rand() / rand();

  this->gyroX = rand() / rand();
  this->gyroY = rand() / rand();
  this->gyroZ = rand() / rand();

  this->temperature = rand() / rand();
  #endif
}

lsm6dso32_accel_range_t Adafruit_LSM6DSO32::getAccelRange(void) {
  return this->_range;
}

void Adafruit_LSM6DSO32::setAccelRange(lsm6dso32_accel_range_t new_range) {
  this->_range = new_range;
}

void Adafruit_LSM6DSO32::show_simulated_file(){
  #ifdef USING_CSV
  sensor_data->head(sensor_data->getSize());
  #endif
}

void Adafruit_LSM6DSO32::increase_file_index(){
  ++(this->file_index);
}
