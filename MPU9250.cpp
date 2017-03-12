#include "MPU9250.h"
#include <math.h>
#include <elapsedMillis.h>
#include "quaternionFilters.h"
#include "i2c.h"
#include <stdint.h>
#include <Arduino.h>
#include "calibration_data.h"


MPU9250::MPU9250(uint8_t address) {
  m_i2cAddress = address;

  aScaleOption = AFS_16G;
  gScaleOption = GFS_2000DPS;
  mScaleOption = MFS_16BITS;

  switch (aScaleOption) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      m_accelMaxValue = 2.0 * sqrt(3.0);
      break;

    case AFS_4G:
      aRes = 4.0 / 32768.0;
      m_accelMaxValue = 4.0 * sqrt(3.0);
      break;

    case AFS_8G:
      aRes = 8.0 / 32768.0;
      m_accelMaxValue = 8.0 * sqrt(3.0);
      break;

    case AFS_16G:
      aRes = 16.0 / 32768.0;
      m_accelMaxValue = 16.0 * sqrt(3.0);
      break;
  }

  switch (gScaleOption) {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      m_gyroMaxValue = 250.0 * sqrt(3.0);
      break;

    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      m_gyroMaxValue = 500.0 * sqrt(3.0);
      break;

    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      m_gyroMaxValue = 1000.0 * sqrt(3.0);
      break;

    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      m_gyroMaxValue = 2000.0 * sqrt(3.0);
      break;
  }

  switch (mScaleOption) {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 4912. / 8190.; // Proper scale to return microteslas
      break;
    case MFS_16BITS:
      mRes = 4912. / 32760.0; // Proper scale to return microteslas
      break;
  }

  m_timestamp = 0;
  m_lastTimestamp = 0;
  m_lastFusionTime = 0;

  m_calibrationMode = false;
  m_gyroCalValid = false;
  m_accelCalValid = false;
  m_magCalValid = false;
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

  writeByte(m_i2cAddress, MPU9250_RA_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(m_i2cAddress, MPU9250_RA_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(m_i2cAddress, MPU9250_RA_GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < N_SAMPLES; ii++) { // get average current values of gyro and acclerometer
    readBytes(m_i2cAddress, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(m_i2cAddress, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of N_SAMPLES values and store as average current readings
    aAvg[ii] /= N_SAMPLES;
    gAvg[ii] /= N_SAMPLES;
  }

  // Configure the accelerometer for self-test
  writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(m_i2cAddress, MPU9250_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

  delay(25);
  //delay(25);  // delay a while to let the device stabilize

  for ( int ii = 0; ii < N_SAMPLES; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(m_i2cAddress, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(m_i2cAddress, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

  }

  for (int ii = 0; ii < 3; ii++) { // Get average of N_SAMPLES values and store as average self-test readings
    aSTAvg[ii] /= N_SAMPLES;
    gSTAvg[ii] /= N_SAMPLES;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG, 0x00);
  writeByte(m_i2cAddress, MPU9250_RA_GYRO_CONFIG,  0x00);
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
  writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x01);
  writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(m_i2cAddress, MPU9250_RA_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(m_i2cAddress, MPU9250_RA_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(m_i2cAddress, MPU9250_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(m_i2cAddress, MPU9250_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(m_i2cAddress, MPU9250_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(m_i2cAddress, MPU9250_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrSensitivity = 131;   // = 131 LSB/degrees/sec
  uint16_t  accSensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0x40);   // Enable FIFO
  writeByte(m_i2cAddress, MPU9250_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  writeByte(m_i2cAddress, MPU9250_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(m_i2cAddress, MPU9250_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (i = 0; i < packet_count; i++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(m_i2cAddress, MPU9250_RA_FIFO_R_W , 12, &data[0]); // read data for averaging
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
  writeByte(m_i2cAddress, MPU9250_RA_XG_OFFS_USRH, data[0]);
  writeByte(m_i2cAddress, MPU9250_RA_XG_OFFS_USRL, data[1]);
  writeByte(m_i2cAddress, MPU9250_RA_YG_OFFS_USRH, data[2]);
  writeByte(m_i2cAddress, MPU9250_RA_YG_OFFS_USRL, data[3]);
  writeByte(m_i2cAddress, MPU9250_RA_ZG_OFFS_USRH, data[4]);
  writeByte(m_i2cAddress, MPU9250_RA_ZG_OFFS_USRL, data[5]);

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
  readBytes(m_i2cAddress, MPU9250_RA_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(m_i2cAddress, MPU9250_RA_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(m_i2cAddress, MPU9250_RA_ZA_OFFSET_H, 2, &data[0]);
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
  writeByte(m_i2cAddress, MPU9250_RA_XA_OFFSET_H, data[0]);
  writeByte(m_i2cAddress, MPU9250_RA_XA_OFFSET_L, data[1]);
  writeByte(m_i2cAddress, MPU9250_RA_YA_OFFSET_H, data[2]);
  writeByte(m_i2cAddress, MPU9250_RA_YA_OFFSET_L, data[3]);
  writeByte(m_i2cAddress, MPU9250_RA_ZA_OFFSET_H, data[4]);
  writeByte(m_i2cAddress, MPU9250_RA_ZA_OFFSET_L, data[5]);
  // Output scaled accelerometer biases for display in the main program
  this->accBias[0] = (float)accel_bias[0] / (float)accSensitivity;
  this->accBias[1] = (float)accel_bias[1] / (float)accSensitivity;
  this->accBias[2] = (float)accel_bias[2] / (float)accSensitivity;
}


bool MPU9250::init() {
  resetSensors();

  while(!testConnection()){
    resetSensors();
    delay(500);
  }

  // Do error checking.
  setGyroConfig();

  setAccelConfig();

  setSampleRate();

  bypassOn();

  extractMagFactoryCal();

  bypassOff();

  // Set up MPU9250 to talk to the compass chip
  setupMagAsSlave();

  enableSensors();

  fifoEnable();

  // Print out contents of registers involved with FIFO enabled
  uint8_t tmp;

  readByte(m_i2cAddress, MPU9250_RA_INT_ENABLE, &tmp);
  Serial.print("MPU9250_RA_INT_ENABLE: "); Serial.println(tmp, BIN);
  readByte(m_i2cAddress, MPU9250_RA_FIFO_EN, &tmp);
  Serial.print("MPU9250_RA_FIFO_EN: "); Serial.println(tmp, BIN);
  readByte(m_i2cAddress, MPU9250_RA_USER_CTRL, &tmp);
  Serial.print("MPU9250_RA_USER_CTRL: "); Serial.println(tmp, BIN);
  readByte(m_i2cAddress, MPU9250_RA_INT_ENABLE, &tmp);
  Serial.print("MPU9250_RA_INT_ENABLE: "); Serial.println(tmp, BIN);

  gyroBiasInit();

  // We are not in calibration mode anymore
  loadCalibrationData();
  setCalibrationMode(false);

  // Gyro bias is not valid.
  m_gyroCalValid = false;
  m_gyroBias = RTVector3();
}

void MPU9250::handleGyroBias() {
  if(!m_calibrationMode) {
    if (!m_gyroCalValid) {
      // nrf_gpio_pin_write(16, 1);

      RTVector3 deltaAccel = m_previousAccel;
      deltaAccel -= m_accel;   // compute difference
      m_previousAccel = m_accel;

      if ((deltaAccel.squareLength() < RTIMU_FUZZY_ACCEL_ZERO_SQUARED) &&
        (m_gyro.squareLength() < RTIMU_FUZZY_GYRO_ZERO_SQUARED)) {
        // what we are seeing on the gyros should be bias only so learn from this
        m_gyroBias.setX((1.0 - m_gyroAlpha) * m_gyroBias.x() + m_gyroAlpha * m_gyro.x());
        m_gyroBias.setY((1.0 - m_gyroAlpha) * m_gyroBias.y() + m_gyroAlpha * m_gyro.y());
        m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_gyroBias.z() + m_gyroAlpha * m_gyro.z());

        if (m_gyroSampleCount < (5 * m_sampleRateGyroAccel)) {
          m_gyroSampleCount++;

          if (m_gyroSampleCount == (5 * m_sampleRateGyroAccel)) {
            m_gyroCalValid = true;

            // nrf_gpio_pin_write(16, 0);
          }
        }
      }
    }

    m_gyro -= m_gyroBias;
  }
}

void MPU9250::handleAccelBias() {
  if(!m_calibrationMode && m_accelCalValid) {
    if(m_accel.x() >= 0) {
      m_accel.setX(m_accel.x() / m_accelMax.x());  
    } else {
      m_accel.setX(m_accel.x() / -m_accelMin.x());  
    }

    if(m_accel.y() >= 0) {
      m_accel.setY(m_accel.y() / m_accelMax.y());  
    } else {
      m_accel.setY(m_accel.y() / -m_accelMin.y());  
    }

    if(m_accel.z() >= 0) {
      m_accel.setZ(m_accel.z() / m_accelMax.z());  
    } else {
      m_accel.setZ(m_accel.z() / -m_accelMin.z());  
    }
  }
}

void MPU9250::handleMagBias() {
  if(!m_calibrationMode && m_magCalValid) {
    m_compass -= m_magOffset;  

    m_compass.setX(m_compass.x() * m_magScale.x());
    m_compass.setY(m_compass.y() * m_magScale.y());
    m_compass.setZ(m_compass.z() * m_magScale.z());
  }
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
  return readByte(m_i2cAddress, MPU9250_RA_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
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
  return readByte(m_i2cAddress, MPU9250_RA_INT_STATUS) & 0x01;
}

/** Read temperature
   Read temperature from MPU and scale to get degrees celcius
   @return Degrees celsius?
*/
int16_t MPU9250::readTempData() {
  uint8_t rawData[2];  // xyz gyro register data stored here
  readBytes(m_i2cAddress, MPU9250_RA_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// bool MPU9250::readAllData() {
//   uint8_t fifoCount[2];
//   uint32_t count;
//   uint8_t fifoData[12];
//   uint8_t compassData[8];

//   if (!readBytes(m_i2cAddress, MPU9250_RA_FIFO_COUNTH, 2, fifoCount)){}
//        // return false;

//   count = ((uint32_t)fifoCount[0] << 8) + fifoCount[1];

//   if (count == 1024) {
//       fifoEnable();
//       return false;
//   }

//   if (count > MPU9250_FIFO_CHUNK_SIZE * 40) {
//       // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
//       while (count >= MPU9250_FIFO_CHUNK_SIZE * 10) {
//           if (!readBytes(m_i2cAddress, MPU9250_RA_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
//               return false;
//           count -= MPU9250_FIFO_CHUNK_SIZE;
//       }
//   }

//   if (count < MPU9250_FIFO_CHUNK_SIZE)
//       return false;

//   if (!readBytes(m_i2cAddress, MPU9250_RA_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
//       return false;

//   // for (int i = 0; i < 12; ++i) {
//   //   Serial.print(fifoData[i]); Serial.print(" ");
//   // }
//   // Serial.println();

//   RTMath::convertToVector(fifoData, m_accel, aRes, BIG_ENDIAN);

//   RTMath::convertToVector(fifoData + 6, m_gyro, gRes, BIG_ENDIAN);

//   if (!readBytes(m_i2cAddress, MPU9250_EXT_SENS_DATA_00, 8, compassData))
//       return false;

//   // First and last bytes out of eight do not contain data?
//   // Magnetometer is set up to read 8 bytes, starting from address 0x02
//   // 0x02 is the ST1 register
//   // 0x03 thru 0x08 is the mag data
//   // 0x09 is the ST2 register
//   // So first and last bytes out of eight contain status data
//   RTMath::convertToVector(compassData + 1, m_compass, mRes, LITTLE_ENDIAN);


//   for (int i = 1; i < 7; ++i) {
//     Serial.print(compassData[i]); Serial.print(" ");
//   }
//   Serial.println();

//   m_compass.setX(m_compass.x() * magASA[0]);
//   m_compass.setY(m_compass.y() * magASA[1]);
//   m_compass.setZ(m_compass.z() * magASA[2]);

//   // Handle axis rotation
//   // The accel and gyro axes are aligned
//   // The compass x-axis is aligned with accel/gyro y-axis
//   // The compass y-axis is aligned with accel/gyro x-axis
//   // The compass z-axis is inverted with respect to accel/gyro z-axis
//   float tmp = m_compass.x();
//   m_compass.setX(m_compass.y());
//   m_compass.setY(tmp);
//   m_compass.setZ(-m_compass.z());

//   m_lastTimestamp = m_timestamp;
//   m_timestamp = elapsedMillis();
//   m_deltat = (m_timestamp - m_lastTimestamp) / 1000.0f;

//   return true;
// }


bool MPU9250::readMagData() {
  unsigned int count;
  unsigned char compassData[8];

  if (!readBytes(m_i2cAddress, MPU9250_EXT_SENS_DATA_00, 8, compassData))
      return false;

  // First and last bytes out of eight do not contain data?
  // Magnetometer is set up to read 8 bytes, starting from address 0x02
  // 0x02 is the ST1 register
  // 0x03 thru 0x08 is the mag data
  // 0x09 is the ST2 register
  // So first and last bytes out of eight contain status data
  RTMath::convertToVector(compassData + 1, m_compass, mRes, LITTLE_ENDIAN);

  m_compass.setX(m_compass.x() * magASA[0]);
  m_compass.setY(m_compass.y() * magASA[1]);
  m_compass.setZ(m_compass.z() * magASA[2]);

  // Handle axis rotation
  // The accel and gyro axes are aligned
  // The compass x-axis is aligned with accel/gyro y-axis
  // The compass y-axis is aligned with accel/gyro x-axis
  // The compass z-axis is inverted with respect to accel/gyro z-axis
  float tmp = m_compass.x();
  m_compass.setX(m_compass.y());
  m_compass.setY(tmp);
  m_compass.setZ(-m_compass.z());

  m_lastTimestamp = m_timestamp;
  m_timestamp = millis();
  m_deltat = (m_timestamp - m_lastTimestamp) / 1000.0f;

  handleMagBias();

  return true;
}

/** Read accelerometer data
   Read 3-axis accelerometer data and update the acc array in MPU9250 instance.
*/
void MPU9250::readAccData() {
  uint8_t rawData[6];  // xyz accel register data stored here
  readBytes(m_i2cAddress, MPU9250_RA_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  this->accData[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  this->accData[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  this->accData[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;

  // Now we'll calculate the accleration value into actual g's
  m_accel.setX((float)this->accData[0] * aRes); // - accelBias[0];
  m_accel.setY((float)this->accData[1] * aRes); // - accelBias[1];
  m_accel.setZ((float)this->accData[2] * aRes); // - accelBias[2];

  handleAccelBias();
}


/** Read gyroscope data
   Read 3-axis gyroscope data and update the gyr array in MPU9250 instance.
*/
void MPU9250::readGyrData() {
  uint8_t rawData[6];  // xyz accel register data stored here
  readBytes(m_i2cAddress, MPU9250_RA_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  this->gyrData[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
  this->gyrData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
  this->gyrData[1] = ((int16_t)rawData[2] << 8) | rawData[3];

  // get actual gyro value, this depends on scale being set
  m_gyro.setX((float)this->gyrData[0] * gRes);
  m_gyro.setY((float)this->gyrData[1] * gRes);
  m_gyro.setZ((float)this->gyrData[2] * gRes);

  handleGyroBias();
}


/** Update filter with newest values from MARG sensors
   Obtain the newest quaternion value by use of a sensor fusion algorithm. Place the newly calculated quaternion values
   in the passed in q array.
*/
void MPU9250::updateFilter(RTQuaternion &q) {
  elapseTime();
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  MadgwickQuaternionUpdate(q, m_accel, m_gyro, m_compass, m_beta, m_zeta, m_deltat);
}


/** Elapse time
*/
void MPU9250::elapseTime() {
  Now = micros();
  m_deltat = (Now - lastUpdate) / 1e6; // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += m_deltat; // sum for averaging filter update rate
}


bool MPU9250::resetSensors() {
  // Reset register and restore default settings.
  if (!writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x80))
    return false;
  delay(100);

  // Use internal 20MHz oscillator as time source.
  if (!writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x00))
    return false;
  delay(20);

  return true;
}

bool MPU9250::enableSensors() {
  if (!writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x01))
    return false;
  delay(10);

  // Use internal 20MHz oscillator as time source.
  if (!writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_2, 0x00))
    return false;
  delay(20);

  return true;
}

bool MPU9250::setGyroConfig() {
  // The lowest two bits of MPU9250_RA_GYRO_CONFIG (reg 0x27) are Fchoice_b,
  // which is an inverted version of Fchoice bits. 
  // Fchoice must be 2'b11 to enable LP filter (so write 2'b00 to Fchoice_b).

  uint8_t gyroConfig = (gScaleOption << 3);
  uint8_t gyroLpf = MPU9250_GYRO_LP & 0x07;

  if (!writeByte(m_i2cAddress, MPU9250_RA_GYRO_CONFIG, gyroConfig))
       return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_CONFIG, gyroLpf))
       return false;

  return true;
}



bool MPU9250::setAccelConfig() {
  // Set full scale range.
  if (!writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG, aScaleOption << 3))
       return false;

  // Set low-pass bandwidth.
  if (!writeByte(m_i2cAddress, MPU9250_RA_ACCEL_CONFIG2, MPU9250_ACCEL_LP))
       return false;
  return true;
}


bool MPU9250::setSampleRate() {
    if (!writeByte(m_i2cAddress, MPU9250_RA_SMPLRT_DIV, (uint8_t) (1000 / m_sampleRateGyroAccel - 1)));
        return false;

    return true;
}


bool MPU9250::bypassOn() {
    uint8_t userControl;

    if (!readByte(m_i2cAddress, MPU9250_RA_USER_CTRL, &userControl))
        return false;

    userControl &= ~0x20;
    userControl |= 2;

    if (!writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!writeByte(m_i2cAddress, MPU9250_RA_INT_PIN_CFG, 0x82))
        return false;

    delay(50);
    return true;
}


bool MPU9250::bypassOff() {
    uint8_t userControl;

    if (!readByte(m_i2cAddress, MPU9250_RA_USER_CTRL, &userControl))
        return false;

    userControl |= 0x20;

    if (!writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!writeByte(m_i2cAddress, MPU9250_RA_INT_PIN_CFG, 0x80))
         return false;

    delay(50);
    return true;
}

bool MPU9250::extractMagFactoryCal() {
  uint8_t buff[3];

  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x00); // Power down magnetometer.
  delay(10);
  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x0F); // Enter Fuse ROM access mode.
  delay(10);

  readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ASAX, 3, &buff[0]);  // Read the xyz-axis calibration values.

  //  Convert asa (sensitivity adjustment) to usable scale factor.
  this->magASA[0] = (float)(buff[0] - 128) / 256. + 1.;
  this->magASA[1] = (float)(buff[1] - 128) / 256. + 1.;
  this->magASA[2] = (float)(buff[2] - 128) / 256. + 1.;

  writeByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_ZA_OFFS_H, 0x00); // Power down magnetometer
  delay(10);

  return true;
}


bool MPU9250::setupMagAsSlave() 
{
  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_MST_CTRL, 0x40))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV0_ADDR, 0x80 | MPU9250_RA_MAG_ADDRESS))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV0_REG, MPU9250_RA_MAG_ST1))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV0_CTRL, 0x88))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV1_ADDR, MPU9250_RA_MAG_ADDRESS))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV1_REG, MPU9250_RA_MAG_CNTL))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV1_CTRL, 0x81))
      return false;

  uint8_t mag_ctrl1_val = (mScaleOption << 4) | 0x6;
  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_SLV1_DO, mag_ctrl1_val))
      return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_I2C_MST_DELAY_CTRL, 0x3))
      return false;

  return true;
}


bool MPU9250::fifoEnable()
{
  if (!writeByte(m_i2cAddress, MPU9250_RA_INT_ENABLE, 0))
    return false;
  if (!writeByte(m_i2cAddress, MPU9250_RA_FIFO_EN, 0))
    return false;
  if (!writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0x04))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_USER_CTRL, 0x60))
    return false;

  delay(50);

  if (!writeByte(m_i2cAddress, MPU9250_RA_INT_ENABLE, 1))
    return false;

  if (!writeByte(m_i2cAddress, MPU9250_RA_FIFO_EN, 0x78))
    return false;

  return true;
}


bool MPU9250::sleepModeEnter() {
  return writeByte(m_i2cAddress, MPU9250_RA_PWR_MGMT_1, 0x40);
}

void MPU9250::gyroBiasInit() {
  m_gyroAlpha = 2.0f / m_sampleRateGyroAccel;
  m_gyroSampleCount = 0;
}

void MPU9250::setCalibrationMode(bool enable) {
  m_calibrationMode = enable;
}


void MPU9250::storeCalibrationData() {
  CalibrationData calData;

  // Gyro calibration values
  calData.gyroCalValid = m_gyroCalValid;
  if(m_gyroCalValid) {
    calData.gyroBias[0] = m_gyroBias.x();
    calData.gyroBias[1] = m_gyroBias.y();
    calData.gyroBias[2] = m_gyroBias.z();
  }

  // Accel calibration values
  calData.accelCalValid = m_accelCalValid;
  if(m_accelCalValid) {
    calData.accelMin[0] = m_accelMin.x();
    calData.accelMin[1] = m_accelMin.y();
    calData.accelMin[2] = m_accelMin.z();

    calData.accelMax[0] = m_accelMax.x();
    calData.accelMax[1] = m_accelMax.y();
    calData.accelMax[2] = m_accelMax.z();
  }

  // Magnetometer calibration values
  calData.magCalValid = m_magCalValid;
  if(m_magCalValid) {
    calData.magOffset[0] = m_magOffset.x();
    calData.magOffset[1] = m_magOffset.y();
    calData.magOffset[2] = m_magOffset.z();

    calData.magScale[0] = m_magScale.x();
    calData.magScale[1] = m_magScale.y();
    calData.magScale[2] = m_magScale.z();
  }

  // clear_calibration_data();
  store_calibration_data(&calData);
}

void MPU9250::loadCalibrationData() {
  calibration_data_init(); // load from EEPROM

  CalibrationData calData;
  load_calibration_data(&calData);

  m_gyroCalValid = (calData.gyroCalValid == 1);

  if(m_gyroCalValid) {
    m_gyroBias.setX(calData.gyroBias[0]);
    m_gyroBias.setY(calData.gyroBias[1]);
    m_gyroBias.setZ(calData.gyroBias[2]);
  }

  m_accelCalValid = (calData.accelCalValid == 1);

  if(m_accelCalValid) {
    m_accelMin.setX(calData.accelMin[0]);
    m_accelMin.setY(calData.accelMin[1]);
    m_accelMin.setZ(calData.accelMin[2]);

    m_accelMax.setX(calData.accelMax[0]);
    m_accelMax.setY(calData.accelMax[1]);
    m_accelMax.setZ(calData.accelMax[2]);
  }

  m_magCalValid = (calData.magCalValid == 1);

  if(m_magCalValid) {
    m_magOffset.setX(calData.magOffset[0]);
    m_magOffset.setY(calData.magOffset[1]);
    m_magOffset.setZ(calData.magOffset[2]);

    m_magScale.setX(calData.magScale[0]);
    m_magScale.setY(calData.magScale[1]);
    m_magScale.setZ(calData.magScale[2]);
  }
}

void MPU9250::clearCalibrationData() {
  CalibrationData calData;

  // Gyro calibration values
  m_gyroCalValid = false;
  calData.gyroCalValid = m_gyroCalValid;

  m_gyroBias = RTVector3(0, 0, 0);
  calData.gyroBias[0] = m_gyroBias.x();
  calData.gyroBias[1] = m_gyroBias.y();
  calData.gyroBias[2] = m_gyroBias.z();

  // Accel calibration values
  m_accelCalValid = false;
  calData.accelCalValid = m_accelCalValid;

  m_accelMin = RTVector3(0, 0, 0);
  m_accelMax = RTVector3(0, 0, 0);
  calData.accelMin[0] = m_accelMin.x();
  calData.accelMin[1] = m_accelMin.y();
  calData.accelMin[2] = m_accelMin.z();
  calData.accelMax[0] = m_accelMax.x();
  calData.accelMax[1] = m_accelMax.y();
  calData.accelMax[2] = m_accelMax.z();

  // Magnetometer calibration values
  m_magCalValid = false;
  calData.magCalValid = m_magCalValid;

  m_magOffset = RTVector3(0, 0, 0);
  m_magScale  = RTVector3(0, 0, 0);
  calData.magOffset[0] = m_magOffset.x();
  calData.magOffset[1] = m_magOffset.y();
  calData.magOffset[2] = m_magOffset.z();
  calData.magScale[0] = m_magScale.x();
  calData.magScale[1] = m_magScale.y();
  calData.magScale[2] = m_magScale.z();

  store_calibration_data(&calData);
}