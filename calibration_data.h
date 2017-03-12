#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <stdint.h>
#include <EEPROM.h>


typedef struct {
  uint8_t gyroCalValid;
  float gyroBias[3];
  uint8_t accelCalValid;
  float accelMin[3];
  float accelMax[3];
  uint8_t magCalValid;
  float magOffset[3];
  float magScale[3];
  float beta;
  float zeta;
  // TODO: ellipsoid calibration data

} CalibrationData;


void calibration_data_init(void) {
  size_t size = sizeof(CalibrationData);
  EEPROM.begin(size);
}

void store_calibration_data(CalibrationData* p_data) {
  size_t size = sizeof(CalibrationData);
  size_t address = 0;

  char *ptr = (char *)p_data;
  while(address < size){
    EEPROM.write(address, ptr[address]);
    address++;
  }

  EEPROM.commit();
}

void load_calibration_data(CalibrationData* p_data) {
  size_t size = sizeof(CalibrationData);
  size_t address = 0;

  char *ptr = (char *) p_data;
  while(address < size){
    ptr[address] = EEPROM.read(address);
    address++;
  }
}

#endif // CALIBRATION_DATA_H