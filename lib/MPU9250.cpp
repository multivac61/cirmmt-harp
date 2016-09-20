#include "MPU9250.h"
#include <math.h>
#include <elapsedMillis.h>
#include "quaternionFilters.h"
#include "i2c.h"
#include <stdint.h>


MPU9250::MPU9250(uint8_t address) {
  devAddr = address;

  // magnitude bias, should update AUTOMATICALLY!!
  this->magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  this->magBias[1] = +120.;  // User environmental y-axis correction in milliGauss
  this->magBias[2] = +125.;  // User environmental z-axis correction in milliGauss

  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }

  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }

  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}


MPU9250::MPU9250() {
  MPU9250(MPU9250_DEFAULT_ADDRESS);
}


void MPU9250::selfTest() {
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;
  uint8_t N_SAMPLES = 200;

  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < N_SAMPLES; ii++) { // get average current values of gyro and acclerometer
    readBytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of N_SAMPLES values and store as average current readings
    aAvg[ii] /= N_SAMPLES;
    gAvg[ii] /= N_SAMPLES;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

  delay(25);
  //delay(25);  // delay a while to let the device stabilize

  for ( int ii = 0; ii < N_SAMPLES; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

  }

  for (int ii = 0; ii < 3; ii++) { // Get average of N_SAMPLES values and store as average self-test readings
    aSTAvg[ii] /= N_SAMPLES;
    gSTAvg[ii] /= N_SAMPLES;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG,  0x00);
  delay(10);

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    this->SelfTestResults[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
    this->SelfTestResults[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
  }
}


/** Report on self test values
   Print the self test values to serial.
*/
void MPU9250::selfTestReport() {
  Serial.print("x-axis self test: acceleration trim within : "); Serial.print(this->SelfTestResults[0], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : "); Serial.print(this->SelfTestResults[1], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : "); Serial.print(this->SelfTestResults[2], 1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : "); Serial.print(this->SelfTestResults[3], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : "); Serial.print(this->SelfTestResults[4], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : "); Serial.print(this->SelfTestResults[5], 1); Serial.println("% of factory value");
}

/** Calibrate gyroscope and accelerometer
   Calculate accelerometer and gyroscope biases by averaging reads from sensors. Store these biases in instance variables.
*/
void MPU9250::calibrate() {
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t i, packet_count, fifo_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_1, 0x01);
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(devAddr, MPU9250_RA_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(devAddr, MPU9250_RA_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(devAddr, MPU9250_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(devAddr, MPU9250_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(devAddr, MPU9250_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(devAddr, MPU9250_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(devAddr, MPU9250_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(devAddr, MPU9250_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(devAddr, MPU9250_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrSensitivity = 131;   // = 131 LSB/degrees/sec
  uint16_t  accSensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(devAddr, MPU9250_RA_USER_CTRL, 0x40);   // Enable FIFO
  writeByte(devAddr, MPU9250_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  writeByte(devAddr, MPU9250_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(devAddr, MPU9250_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (i = 0; i < packet_count; i++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(devAddr, MPU9250_RA_FIFO_R_W , 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0]  /= (int32_t)packet_count;
  gyro_bias[1]  /= (int32_t)packet_count;
  gyro_bias[2]  /= (int32_t)packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accSensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  } else {
    accel_bias[2] += (int32_t) accSensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(devAddr, MPU9250_RA_XG_OFFS_USRH, data[0]);
  writeByte(devAddr, MPU9250_RA_XG_OFFS_USRL, data[1]);
  writeByte(devAddr, MPU9250_RA_YG_OFFS_USRH, data[2]);
  writeByte(devAddr, MPU9250_RA_YG_OFFS_USRL, data[3]);
  writeByte(devAddr, MPU9250_RA_ZG_OFFS_USRH, data[4]);
  writeByte(devAddr, MPU9250_RA_ZG_OFFS_USRL, data[5]);

  // Output scaled gyro biases for display in the main program
  this->gyrBias[0] = (float)gyro_bias[0] / (float)gyrSensitivity;
  this->gyrBias[1] = (float)gyro_bias[1] / (float)gyrSensitivity;
  this->gyrBias[2] = (float)gyro_bias[2] / (float)gyrSensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.
  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(devAddr, MPU9250_RA_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(devAddr, MPU9250_RA_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(devAddr, MPU9250_RA_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (i = 0; i < 3; i++) {
    if ((accel_bias_reg[i] & mask)) {
      mask_bit[i] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
  }
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(devAddr, MPU9250_RA_XA_OFFSET_H, data[0]);
  writeByte(devAddr, MPU9250_RA_XA_OFFSET_L, data[1]);
  writeByte(devAddr, MPU9250_RA_YA_OFFSET_H, data[2]);
  writeByte(devAddr, MPU9250_RA_YA_OFFSET_L, data[3]);
  writeByte(devAddr, MPU9250_RA_ZA_OFFSET_H, data[4]);
  writeByte(devAddr, MPU9250_RA_ZA_OFFSET_L, data[5]);
  // Output scaled accelerometer biases for display in the main program
  this->accBias[0] = (float)accel_bias[0] / (float)accSensitivity;
  this->accBias[1] = (float)accel_bias[1] / (float)accSensitivity;
  this->accBias[2] = (float)accel_bias[2] / (float)accSensitivity;
}


/** Initialize gyroscope and accelerometer
   Activate continuous read of gyroscope and accelerometer at highest resolution
   @return void
*/
void MPU9250::calibrate_mag() {
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int32_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

  // shoot for ~fifteen seconds of mag data
  if (this->Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  if (this->Mmode == 0x06) sample_count = 2; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    readMagData();  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (this->mag[jj] > mag_max[jj]) mag_max[jj] = this->mag[jj];
      if (this->mag[jj] < mag_min[jj]) mag_min[jj] = this->mag[jj];
    }
    if (this->Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (this->Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  // Added minus signes due to chip placement
  this->magBias[0] = -(float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  this->magBias[1] = -(float) mag_bias[1] * mRes * magCalibration[1];
  this->magBias[2] = -(float) mag_bias[2] * mRes * magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  this->magScale[0] = avg_rad / ((float)mag_scale[0]);
  this->magScale[1] = avg_rad / ((float)mag_scale[1]);
  this->magScale[2] = avg_rad / ((float)mag_scale[2]);
}



/** Initialize gyroscope and accelerometer
   Activate continuous read of gyroscope and accelerometer at highest resolution
   @return void
*/
void MPU9250::init() {
  // wake up device
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(10); // Delay for all registers to reset

  // get stable time source
  writeByte(devAddr, MPU9250_RA_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(20);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(devAddr, MPU9250_RA_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + MPU9250_RA_SMPLRT_DIV)
  writeByte(devAddr, MPU9250_RA_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in MPU9250_RA_CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(devAddr, MPU9250_RA_GYRO_CONFIG); // get current MPU9250_RA_GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of MPU9250_RA_GYRO_CONFIG
  writeByte(devAddr, MPU9250_RA_GYRO_CONFIG, c ); // Write new MPU9250_RA_GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(devAddr, MPU9250_RA_ACCEL_CONFIG); // get current MPU9250_RA_ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(devAddr, MPU9250_RA_ACCEL_CONFIG, c); // Write new MPU9250_RA_ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(devAddr, MPU9250_RA_ACCEL_CONFIG2); // get current MPU9250_RA_ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(devAddr, MPU9250_RA_ACCEL_CONFIG2, c); // Write new MPU9250_RA_ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the MPU9250_RA_SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(devAddr, MPU9250_RA_INT_PIN_CFG, 0x22);
  writeByte(devAddr, MPU9250_RA_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(10);

  // Finally we initialize the magnetometer
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x00); // Power down magnetometer
  delay(10);
  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  this->magCalibration[0] = (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  this->magCalibration[1] = (float)(rawData[1] - 128) / 256. + 1.;
  this->magCalibration[2] = (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


/** Verify the I2C connection with MPU9250
   Make sure the device is connected and responds as expected.
   @return True if connection is valid, false otherwise
*/
bool MPU9250::testConnection() {
  // Read the WHO_AM_I register, this is a good test of communication
  return getDeviceID() == MPU9250_DEV_ID;
}


/** Verify the I2C connection with AK
   Make sure the magnetometer is connected and responds as expected.
   @return True if connection is valid, false otherwise
*/
bool MPU9250::testMagConnection() {
  // Read the WHO_AM_I register, this is a good test of communication
  return getMagID() == MPU9250_MAG_ID;
}


/** Get Device ID.
   This register is used to verify the identity of the device (0b110100, 0x34).
   @return Device ID or 0xFF if an error occured when reading the ID.
*/
uint8_t MPU9250::getDeviceID() {
  return readByte(devAddr, MPU9250_RA_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
}


/** Get Magnetometer ID.
   This register is used to verify the identity of the magnetometer
   @return Device ID or 0xFF if an error occured when reading the ID.
*/
uint8_t MPU9250::getMagID() {
  return readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
}


/** See if new measurements are available
   This register is used to verify the identity of the magnetometer
   @return True if new data is available from sensor, false otherwise.
*/
uint8_t MPU9250::isInterrupted() {
  // If INT_STATUS is high, all data registers have new data
  return readByte(devAddr, MPU9250_RA_INT_STATUS) & 0x01;
}


/** Read accelerometer data
   Read 3-axis accelerometer data and update the acc array in MPU9250 instance.
*/
void MPU9250::readAccData() {
  uint8_t rawData[6];  // xyz accel register data stored here
  readBytes(devAddr, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  this->accData[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  this->accData[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  this->accData[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;

  // Now we'll calculate the accleration value into actual g's
  this->acc[0] = (float)this->accData[0] * aRes; // - accelBias[0];
  this->acc[1] = (float)this->accData[1] * aRes; // - accelBias[1];
  this->acc[2] = (float)this->accData[2] * aRes; // - accelBias[2];
}


/** Read gyroscope data
   Read 3-axis gyroscope data and update the gyr array in MPU9250 instance.
*/
void MPU9250::readGyrData() {
  uint8_t rawData[6];  // xyz accel register data stored here
  readBytes(devAddr, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  this->gyrData[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
  this->gyrData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
  this->gyrData[1] = ((int16_t)rawData[2] << 8) | rawData[3];

  // get actual gyro value, this depends on scale being set
  this->gyr[0] = (float)this->gyrData[0] * gRes;
  this->gyr[1] = (float)this->gyrData[1] * gRes;
  this->gyr[1] = (float)this->gyrData[1] * gRes;
}


/** Read magnetometer data
   Read 3-axis magnetometer data and update the mag array in MPU9250 instance.
*/
void MPU9250::readMagData() {
  uint8_t rawData[7];  // xyz gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  // delay(10);
  // Serial.println(readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1));
  // delay(10);
  // Serial.println(readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1) & 0x01);
  if (readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1) & 0x01) { // delay for magnetometer data ready bit to be set
    // delay(10);
    readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    // Serial.println("Printing rawData[0,1,6]");
    // Serial.println(rawData[0]);
    // Serial.println(rawData[1]);
    // Serial.println(rawData[6]);
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      this->magData[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
      this->magData[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
      this->magData[2] = ((int16_t)rawData[5] << 8) | rawData[4];

      // get actual magnetometer value, this depends on scale being set
      this->mag[0] = (float)this->magData[0] * mRes * magCalibration[0] - magBias[0];
      this->mag[1] = (float)this->magData[1] * mRes * magCalibration[1] - magBias[1];
      this->mag[2] = (float)this->magData[2] * mRes * magCalibration[2] - magBias[2];
    }
  }
}


/** Read temperature
   Read temperature from MPU and scale to get degrees celcius
   @return Degrees celsius?
*/
int16_t MPU9250::readTempData() {
  uint8_t rawData[2];  // xyz gyro register data stored here
  readBytes(devAddr, MPU9250_RA_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


/** Read data from MARG sensors
   Read temperature from MPU and scale to get degrees celcius
*/
void MPU9250::readAllData() {
  readAccData();
  readGyrData();
  readMagData();
}


/** Update filter with newest values from MARG sensors
   Obtain the newest quaternion value by use of a sensor fusion algorithm. Place the newly calculated quaternion values
   in the passed in q array.
*/
void MPU9250::updateFilter(float* q) {
  elapseTime();
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  MahonyQuaternionUpdate(q, this->acc, this->gyr, this->mag, deltat);
}


/** Elapse time
*/
void MPU9250::elapseTime() {
  Now = micros();
  deltat = (Now - lastUpdate) / 1000000.0f; // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
}