// Wire.h read and write protocols
#include <stdint.h>
#include <Wire.h>

bool writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
bool readByte(uint8_t address, uint8_t subAddress, uint8_t *data);
bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);