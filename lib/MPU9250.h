#include <stdint.h>
#include "RTMath.h"

// Register variables
#define MPU9250_DEV_ID              0x71
#define MPU9250_MAG_ID              0x48

// Magnetometer Registers
#define MPU9250_RA_MAG_ADDRESS      0x0C
#define MPU9250_RA_MAG_WHO_AM_I     0x00
#define MPU9250_RA_MAG_INFO         0x02
#define MPU9250_RA_MAG_ST1          0x02
#define MPU9250_RA_MAG_XOUT_L       0x03
#define MPU9250_RA_MAG_XOUT_H       0x04
#define MPU9250_RA_MAG_YOUT_L       0x05
#define MPU9250_RA_MAG_YOUT_H       0x06
#define MPU9250_RA_MAG_ZOUT_L       0x07
#define MPU9250_RA_MAG_ZOUT_H       0x08
#define MPU9250_RA_MAG_ASAX         0x10 // Fuse ROM x-axis sensitivity adjustment value
#define MPU9250_RA_MAG_ASAY         0x11
#define MPU9250_RA_MAG_ASAZ         0x12

#define MPU9250_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS     MPU9250_ADDRESS_AD0_LOW

#define MPU9250_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU9250_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU9250_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU9250_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x07
#define MPU9250_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x09
#define MPU9250_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x0B
#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RA_ZG_OFFS_USRL     0x18
#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_ACCEL_CONFIG2    0x1D
#define MPU9250_RA_FF_DUR           0x1E
#define MPU9250_RA_MOT_THR          0x1F
#define MPU9250_RA_MOT_DUR          0x20
#define MPU9250_RA_ZRMOT_THR        0x21
#define MPU9250_RA_ZRMOT_DUR        0x22
#define MPU9250_RA_FIFO_EN          0x23
#define MPU9250_RA_I2C_MST_CTRL     0x24
#define MPU9250_RA_INT_PIN_CFG      0x37
#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_DMP_INT_STATUS   0x39
#define MPU9250_RA_INT_STATUS       0x3A
#define MPU9250_RA_ACCEL_XOUT_H     0x3B
#define MPU9250_RA_ACCEL_XOUT_L     0x3C
#define MPU9250_RA_ACCEL_YOUT_H     0x3D
#define MPU9250_RA_ACCEL_YOUT_L     0x3E
#define MPU9250_RA_ACCEL_ZOUT_H     0x3F
#define MPU9250_RA_ACCEL_ZOUT_L     0x40
#define MPU9250_RA_TEMP_OUT_H       0x41
#define MPU9250_RA_TEMP_OUT_L       0x42
#define MPU9250_RA_GYRO_XOUT_H      0x43
#define MPU9250_RA_GYRO_XOUT_L      0x44
#define MPU9250_RA_GYRO_YOUT_H      0x45
#define MPU9250_RA_GYRO_YOUT_L      0x46
#define MPU9250_RA_GYRO_ZOUT_H      0x47
#define MPU9250_RA_GYRO_ZOUT_L      0x48
#define MPU9250_RA_MOT_DETECT_STATUS    0x61
#define MPU9250_RA_I2C_SLV0_DO      0x63
#define MPU9250_RA_I2C_SLV1_DO      0x64
#define MPU9250_RA_I2C_SLV2_DO      0x65
#define MPU9250_RA_I2C_SLV3_DO      0x66
#define MPU9250_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_RA_SIGNAL_PATH_RESET    0x68
#define MPU9250_RA_MOT_DETECT_CTRL      0x69
#define MPU9250_RA_USER_CTRL        0x6A
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_PWR_MGMT_2       0x6C
#define MPU9250_RA_BANK_SEL         0x6D
#define MPU9250_RA_MEM_START_ADDR   0x6E
#define MPU9250_RA_MEM_R_W          0x6F
#define MPU9250_RA_DMP_CFG_1        0x70
#define MPU9250_RA_DMP_CFG_2        0x71
#define MPU9250_RA_FIFO_COUNTH      0x72
#define MPU9250_RA_FIFO_COUNTL      0x73
#define MPU9250_RA_FIFO_R_W         0x74
#define MPU9250_RA_WHO_AM_I         0x75
#define MPU9250_RA_XA_OFFSET_H      0x77
#define MPU9250_RA_XA_OFFSET_L      0x78
#define MPU9250_RA_YA_OFFSET_H      0x7A
#define MPU9250_RA_YA_OFFSET_L      0x7B
#define MPU9250_RA_ZA_OFFSET_H      0x7D
#define MPU9250_RA_ZA_OFFSET_L      0x7E

// Gyroscope Registers
#define MPU9250_GCONFIG_FS_SEL_BIT      4
#define MPU9250_GCONFIG_FS_SEL_LENGTH   2

#define MPU9250_GYRO_FS_250         0x00
#define MPU9250_GYRO_FS_500         0x01
#define MPU9250_GYRO_FS_1000        0x02
#define MPU9250_GYRO_FS_2000        0x03

// Accelerometer Registers
#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          0x01
#define MPU9250_ACCEL_FS_8          0x02
#define MPU9250_ACCEL_FS_16         0x03

#define MPU9250_RA_SELF_TEST_X_GYRO 0x00
#define MPU9250_RA_SELF_TEST_Y_GYRO 0x01
#define MPU9250_RA_SELF_TEST_Z_GYRO 0x02

#define MPU9250_RA_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_RA_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_RA_SELF_TEST_Z_ACCEL 0x0F

// I'm pretty sure the RTIMU library was expecting gyro data in radians
// Using the figure of 0.20 the device never gathers enough data to
// complete gyro bias calculation.
// Treating the 0.20 figure as radians, the gyro bias calculation is OK
// This defines the gyroscope noise level
#define RTIMU_FUZZY_GYRO_ZERO           (RTFLOAT)0.20 * 180 / RTMATH_PI
#define RTIMU_FUZZY_GYRO_ZERO_SQUARED   (RTIMU_FUZZY_GYRO_ZERO * RTIMU_FUZZY_GYRO_ZERO)

// This defines the accelerometer noise level
#define RTIMU_FUZZY_ACCEL_ZERO          (RTFLOAT)0.05
#define RTIMU_FUZZY_ACCEL_ZERO_SQUARED (RTIMU_FUZZY_ACCEL_ZERO * RTIMU_FUZZY_ACCEL_ZERO)


// Options
enum ASCALE {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum GSCALE {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum MSCALE {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

void initMPU9250(uint8_t Gscale, uint8_t Ascale);
void calibrateMPU9250(float *dest1, float *dest2);
void MPU9250SelfTest(float *dest);

void initAK8963(float* destination, uint8_t Mscale, uint8_t Mmode);

void readAccelData(int16_t *destination);
void readGyroData(int16_t *destination);
void readMagData(int16_t *destination);
int16_t readTempData();

class MPU9250 {
      private:
        uint8_t devAddr;             // Device I2C address
        uint8_t buffer[14];          // Data read buffer

        // variables for timing
        float    deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
        uint32_t lastUpdate = 0, firstUpdate = 0;  // used to calculate integration interval
        uint32_t Now = 0;                          // used to calculate integration interval
        uint8_t  Gscale = GFS_2000DPS;
        uint8_t  Ascale = AFS_8G;
        uint8_t  Mscale = MFS_14BITS; // Choose either 14-bit or 16-bit magnetometer resolution
        uint8_t  Mmode = 0x06;        // WAS 2!! 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
        float    aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

        int16_t accData[3];          // Stores the 16-bit signed accelerometer sensor output
        int16_t gyrData[3];          // Stores the 16-bit signed gyro sensor output
        int16_t magData[3];          // Stores the 16-bit signed magnetometer sensor output
        float   gyrBias[3] = {0, 0, 0},  // Bias corrections for gyro and accelerometer
                accBias[3] = {0, 0, 0};         
        int16_t tempData;            // temperature raw count output
        float   temperature;         // Stores the real internal chip temperature in Celsius
        float   SelfTest[6];         // holds results of gyro and accelerometerself test
        float   SelfTestResults[6] = {0, 0, 0, 0, 0, 0};

        float m_deltat, m_lastTimestamp, m_timestamp;
        
        
      public:
        float acc[3] = {0, 0, 0};
        float gyr[3] = {0, 0, 0};
        float mag[3] = {0, 0, 0};

        float magCalibration[3] = {0, 0, 0}, 
              magBias[3]        = {0, 0, 0}, 
              magScale[3]       = {0, 0, 0};


        RTVector3 m_accel, m_gyro, m_compass;
        RTVector3 m_accelMin, m_accelMax, m_magOffset, m_magScale,  m_gyroBias, m_previousAccel;
        float m_gyroAlpha;
        uint16_t m_gyroSampleCount;

        bool m_magCalValid, m_gyroCalValid, m_calibrationMode, m_accelCalValid;

        int m_sampleRateGyroAccel = 100;

        MPU9250();
        MPU9250(uint8_t address);

        void init();
        void calibrate();
        bool testConnection();
        bool testMagConnection();
        void calibrate_mag();

        // Self testing
        void selfTest();
        void selfTestReport();

        uint8_t isInterrupted();

        void readAllData();
        void readAccData();
        void readGyrData();
        void readMagData();
        int16_t readTempData();

        void elapseTime();
        void updateFilter(float* q);

        // Calibrate
        void handleGyroBias();
        void handleAccelBias();
        void handleMagBias();

        // AUX_VDDIO register
        uint8_t getAuxVDDIOLevel();
        void setAuxVDDIOLevel(uint8_t level);

        // GYRO_CONFIG register
        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);

        // WHO_AM_I register
        uint8_t getDeviceID();
        uint8_t getMagID();
        void setDeviceID(uint8_t id);
};