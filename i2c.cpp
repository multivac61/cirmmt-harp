// Wire.h read and write protocols
#include "i2c.h"

bool writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  
  Wire.beginTransmission (address);  // Initialize the Tx buffer
  
  int bytes_written = Wire.write(subAddress);           // Put slave register address in Tx buffer
  if(bytes_written < 0){
    return false;
  }

  bytes_written = Wire.write(data);                 // Put data in Tx buffer
  if(bytes_written < 0){
    return false;
  }

  Wire.endTransmission();           // Send the Tx buffer
  return true;
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

bool readByte(uint8_t address, uint8_t subAddress, uint8_t *data)
{
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  int bytes_written = Wire.write(subAddress);                  // Put slave register address in Tx buffer
  if(bytes_written < 0){
    return false;
  }

  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  *data = Wire.read();                      // Fill Rx buffer with result
  
  return true;
}

bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  int bytes_written = Wire.write(subAddress);            // Put slave register address in Tx buffer
  if(bytes_written < 0){
    return false;
  }

  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  int bytes_received = Wire.requestFrom(address, count);  // Read bytes from slave register address
  if(bytes_received < count){
    return false;
  }

  while (Wire.available()) {
    dest[i++] = Wire.read();
  }

  return true;
}
