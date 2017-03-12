#include <Wire.h>
#include "MPU9250.h"
#include "Adafruit_DRV2605.h"

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>

#include "OSCMessage.h"
#include "OSCData.h"
#include "RTMath.h"


extern "C" {
#include "user_interface.h"
}

#define SERIAL_DEBUG true      // Print initialization messages over serial
#define SERIAL_PROC  true      // Print data messages over serial
#define SEND_CALIBRATION false // Send calibration constants over UDP
#define WAIT_WIFI 10           // UDP send interval

#ifdef ARDUINO_ESP8266_ESP01 // CIRMMT HARP
  #define SDA 12
  #define SCL 13
  #define RED_LED 5
  #define GREEN_LED 4
  #define BLUE_LED 15
#elif  ARDUINO_ESP8266_ESP12 // HUZZA
  #define SDA 12
  #define SCL 13
  #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0_LOW
  // #define SDA 4
  // #define SCL 5
  // #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0_LOW
  #define RED_LED 5
  #define GREEN_LED 4
  #define BLUE_LED 15
#else
  #define SDA 12
  #define SCL 13
  // #define SDA 4
  // #define SCL 5
  #define RED_LED 5
  #define GREEN_LED 4
  #define BLUE_LED 15
#endif

// RECEIVER CODES
enum RECEIVE
{
  RECEIVE_HAPTIC             = 'd',
  RECEIVE_LED                = 'l',
  RECEIVE_RECENTER           = 'r',
  RECEIVE_BETA_ZETA          = 'b',
  RECEIVE_HANDLE_CALIBRATION = 'h',
  RECEIVE_ACCEL_SENSOR_ERROR = 'a',
  RECEIVE_GYRO_SENSOR_ERROR  = 'g',
  RECEIVE_MAG_SENSOR_ERROR   = 'm',
  RECEIVE_CLEAR_CALIBRATION  = 'c'
};

// Initialization of software interrupt functions
os_timer_t wifiTimer;
os_timer_t filterTimer;

bool interrupt_wifi;

void timerCallback(void *pArg) {
  interrupt_wifi = true;
}

void user_init(void) {
  os_timer_setfn(&wifiTimer, timerCallback, NULL);
  os_timer_arm(&wifiTimer, WAIT_WIFI, true);
}

// MPU9250 imu(MPU9250_ADDRESS_AD0_HIGH);
MPU9250 imu(MPU9250_ADDRESS);

// Wifi configuration
WiFiUDP udp;

// Define the SSID and PASSWORD of network
const char* SSID = "bodysuit";
const char* PASSWORD = "bodysuit";

// Unique identifier for given chip.
const int LAST_IP_HOST = 141;
const int LAST_IP_CLIENT = 111;

IPAddress home(192, 168, 1, LAST_IP_HOST); 
IPAddress broadcast(192, 168, 1, 255);
IPAddress mask(255, 255, 255, 0);

#define LISTEN_PORT 5001  // Port for incoming OSC messages
#define SEND_PORT 50000 + LAST_IP_HOST    // Port for outgoing OSC messages

// Static IP of receiving client
IPAddress sendIP(192, 168, 1, LAST_IP_CLIENT);
// IPAddress sendIP(192, 168, 10, 10Float

// Global variable for haptic motor
// 192.168.10.44
Adafruit_DRV2605 drv;

// Onboard RGB diode pins

// RTQuaternion for recentering
RTQuaternion q_recenter(1.0, 0.0, 0.0, 0.0), q_corr;
RTQuaternion q(1.0, 0.0, 0.0, 0.0);
RTVector3 eulerAngles;

// Utility functions
void printMessage(OSCMessage &msg){
  char address[255];
  int len = msg.getAddress(address, 0);
  for(int i=0; i<len; i++){
     Serial.print(address[i]);
  }
   
  Serial.print(" with data ");
  float data = msg.getInt(0);
  Serial.print(data);
  Serial.println(" ");
}

void setColor(int red, int green, int blue){
  analogWrite(RED_LED, red);
  analogWrite(GREEN_LED, green);
  analogWrite(BLUE_LED, blue);  
}

unsigned long time_now, time_past;
unsigned long time_now_loop, time_past_loop;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin(SDA, SCL);
  // Wire.setClock(400e3);
  
  Serial.begin(38400);
  delay(500);

  while (!imu.testConnection()) {
    if (SERIAL_DEBUG) {
      Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(imu.getDeviceID(), HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
      Serial.println(". Could not connect to MPU9250");
    }
    delay(100);
  }

  if (SERIAL_DEBUG) Serial.println("MPU9250 is online...");

  imu.init();

  if (SERIAL_DEBUG) Serial.println("MPU9250 initialized for active data mode....");

  if (SERIAL_DEBUG) {
    Serial.print("Magdwick beta, zeta values:");
    Serial.print(imu.beta()); Serial.print(" ");
    Serial.print(imu.zeta()); Serial.println();

    Serial.print("Accel sensor error model.");
    Serial.print("Enabled: "); Serial.print(imu.m_accelCalValid);
    Serial.print(" Values: ");
    Serial.print(imu.m_accelMin.x()); Serial.print(" ");
    Serial.print(imu.m_accelMax.x()); Serial.print(" ");
    Serial.print(imu.m_accelMin.y()); Serial.print(" ");
    Serial.print(imu.m_accelMax.y()); Serial.print(" ");
    Serial.print(imu.m_accelMin.z()); Serial.print(" ");
    Serial.print(imu.m_accelMax.z()); Serial.println();

    Serial.print("Compass sensor error model.");
    Serial.print("Enabled: "); Serial.print(imu.m_magCalValid);
    Serial.print(" Values: ");
    Serial.print(imu.m_magOffset.x()); Serial.print(" ");
    Serial.print(imu.m_magScale.x());  Serial.print(" ");
    Serial.print(imu.m_magOffset.y()); Serial.print(" ");
    Serial.print(imu.m_magScale.y());  Serial.print(" ");
    Serial.print(imu.m_magOffset.z()); Serial.print(" ");
    Serial.print(imu.m_magScale.z());  Serial.println();

    Serial.print("Gyro sensor error model.");
    Serial.print("Enabled: "); Serial.print(imu.m_gyroCalValid);
    Serial.print(" Values: ");
    Serial.print(imu.m_gyroBias.x()); Serial.print(" ");
    Serial.print(imu.m_gyroBias.y()); Serial.print(" ");
    Serial.print(imu.m_gyroBias.z()); Serial.println();
  }

  // Setup the wifi connection
  WiFi.disconnect();
  WiFi.begin(SSID, PASSWORD);         // Connect to Wifi network
  WiFi.config(home, broadcast, mask); // Obtain a static IP address

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    if (SERIAL_DEBUG) {
      Serial.print(".");
    }
  }

  digitalWrite(LED_BUILTIN, HIGH); // Turn on notification light

  if (SERIAL_DEBUG) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  udp.begin(LISTEN_PORT);

  if (SERIAL_DEBUG) {
    Serial.println("Starting UDP");
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
    Serial.print("Remote port: ");
    Serial.println(SEND_PORT);
    Serial.println("Setup complete!");
  }

  drv.begin(SDA, SCL);
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); 

  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);

  // Software interrupts
  interrupt_wifi = false;
  user_init();
}

void loop() {
  int size = udp.parsePacket();
  if(size > 0) {
    OSCMessage incoming_osc;
    while(size--){
      incoming_osc.fill(udp.read());
    }
    if(SERIAL_PROC){
      printMessage(incoming_osc);
    }
    
    char address[64];
    int len = incoming_osc.getAddress(address, 0);

    char id = address[1];
    switch(id){
      case RECEIVE_HAPTIC:{
        int end = 0;
        for(int i = 0; i < 8; i++){
          int effect = incoming_osc.getInt(i);
          if(effect < 0){
            end = i;
            break;
          }

          drv.setWaveform(i, effect);
          end = i+1;
        }

        drv.setWaveform(end, 0);
        drv.go();
        break;
      }

      case RECEIVE_LED:{
        int r = incoming_osc.getInt(0);
        int g = incoming_osc.getInt(1);
        int b = incoming_osc.getInt(2);
        setColor(r, g, b);
        break;
      }

      case RECEIVE_RECENTER:{
        RTQuaternion q0 = q;
        RTQuaternion q1 = RTQuaternion(1, 0, 0, 0);
        q0 = RTQuaternion(q0.scalar(), -q0.x(), -q0.y(), -q0.z());
        q1 *= q0;

        q_recenter = q1;

        if (SERIAL_DEBUG) {
          Serial.println("Recentered quaternion on device.");
        }
        break;
      }

      case RECEIVE_BETA_ZETA:{
        float beta = incoming_osc.getFloat(0);
        float zeta = incoming_osc.getFloat(1);

        imu.setBeta(beta);
        imu.setBeta(zeta);

        if (SERIAL_DEBUG) {
          Serial.print("Recentered beta, zeta values: ");
          Serial.print(beta); Serial.print(", ");
          Serial.print(zeta); Serial.println("");
        }

        imu.storeCalibrationData();
        break;
      }

      case RECEIVE_ACCEL_SENSOR_ERROR:{
        float accel_x_min = incoming_osc.getFloat(0);
        float accel_x_max = incoming_osc.getFloat(1);
        float accel_y_min = incoming_osc.getFloat(2);
        float accel_y_max = incoming_osc.getFloat(3);
        float accel_z_min = incoming_osc.getFloat(4);
        float accel_z_max = incoming_osc.getFloat(5);

        imu.m_accelMax = RTVector3(accel_x_max, accel_y_max, accel_z_max);
        imu.m_accelMin = RTVector3(accel_x_min, accel_y_min, accel_z_min);
        imu.m_accelCalValid = true;

        if (SERIAL_DEBUG) {
          Serial.print("Received accel sensor error model:");
          Serial.print(imu.m_accelMin.x()); Serial.print(" ");
          Serial.print(imu.m_accelMax.x()); Serial.print(" ");
          Serial.print(imu.m_accelMin.y()); Serial.print(" ");
          Serial.print(imu.m_accelMax.y()); Serial.print(" ");
          Serial.print(imu.m_accelMin.z()); Serial.print(" ");
          Serial.print(imu.m_accelMax.z()); Serial.println();
        }

        imu.storeCalibrationData();
        break;
      }

      case RECEIVE_GYRO_SENSOR_ERROR:{
        float gyro_x_bias = incoming_osc.getFloat(0);
        float gyro_y_bias = incoming_osc.getFloat(1);
        float gyro_z_bias = incoming_osc.getFloat(2);

        if (SERIAL_DEBUG) {
          Serial.print("Received gyro sensor error model: ");
          Serial.print(gyro_x_bias); Serial.print(" ");
          Serial.print(gyro_y_bias); Serial.print(" ");
          Serial.print(gyro_z_bias); Serial.println();
        }

        imu.m_gyroBias = RTVector3(gyro_x_bias, gyro_y_bias, gyro_z_bias);
        imu.m_gyroCalValid = true;

        imu.storeCalibrationData();
        break;
      }

      case RECEIVE_MAG_SENSOR_ERROR:{
        float mag_x_offset = incoming_osc.getFloat(0);
        float mag_x_scale  = incoming_osc.getFloat(1);
        float mag_y_offset = incoming_osc.getFloat(2);
        float mag_y_scale  = incoming_osc.getFloat(3);
        float mag_z_offset = incoming_osc.getFloat(4);
        float mag_z_scale  = incoming_osc.getFloat(5);

        if (SERIAL_DEBUG) {
          Serial.print("Received mag sensor error model: ");
          Serial.print(mag_x_offset); Serial.print(" ");
          Serial.print(mag_x_scale);  Serial.print(" ");
          Serial.print(mag_y_offset); Serial.print(" ");
          Serial.print(mag_y_scale);  Serial.print(" ");
          Serial.print(mag_z_offset); Serial.print(" ");
          Serial.print(mag_z_scale);  Serial.println();
        }

        imu.m_magOffset = RTVector3(mag_x_offset, mag_y_offset, mag_z_offset);
        imu.m_magScale  = RTVector3(mag_x_scale, mag_y_scale, mag_z_scale);
        imu.m_magCalValid = true;

        imu.storeCalibrationData();
        break;
      }

      case RECEIVE_HANDLE_CALIBRATION:{
        int enable_int = incoming_osc.getInt(0);
        bool enable = enable_int == 1 ? true : false;

        if (SERIAL_DEBUG) {
          if(enable) {
            Serial.println("Calibration mode enabled");
          } else {
            Serial.println("Calibration mode disabled");
          }
        }

        imu.m_calibrationMode = enable;
        break;
      }

      case RECEIVE_CLEAR_CALIBRATION:{
        imu.clearCalibrationData();

        if (SERIAL_DEBUG) {
          Serial.println("Calibration data cleared");
        }

        break;
      }

    }
  }
  
  if (SERIAL_PROC) {
    time_now_loop = millis();
    Serial.print("Between loop: "); Serial.println(time_now_loop - time_past_loop);
    time_past_loop = time_now_loop;
  }
  
  if (imu.isInterrupted()) {
    bool read_data = imu.readMagData();

    // Data stored in fifo buffer is utter trash. Need to read accel/gyro data 
    // directly from raw data registers on the MPU9250.
    imu.readAccData();
    imu.readGyrData();

    if (SERIAL_PROC) {
      time_now = millis();
      Serial.print("Between readings: "); Serial.println(time_now   - time_past);
      time_past = time_now;
    }
    
    if(SERIAL_PROC && !read_data){
      Serial.println("Didn't read data.");
    }

    if (SERIAL_PROC) {
      Serial.print("Accel: ");
      Serial.print(imu.m_accel.x(), 6); Serial.print(" ");
      Serial.print(imu.m_accel.y(), 6); Serial.print(" ");
      Serial.println(imu.m_accel.z(), 6);

      Serial.print("Gyro: ");
      Serial.print(imu.m_gyro.x(), 6); Serial.print(" ");
      Serial.print(imu.m_gyro.y(), 6); Serial.print(" ");
      Serial.println(imu.m_gyro.z(), 6);

      Serial.print("Compass: ");
      Serial.print(imu.m_compass.x(), 6); Serial.print(" ");
      Serial.print(imu.m_compass.y(), 6); Serial.print(" ");
      Serial.println(imu.m_compass.z(), 6);

      Serial.print("Quaternion: ");
      Serial.print(q.scalar(), 6);   Serial.print(" ");
      Serial.print(q.x(), 6); Serial.print(" ");
      Serial.print(q.y(), 6); Serial.print(" ");
      Serial.println(q.z(), 6);
    }
    
    imu.updateFilter(q);
  }

  if (interrupt_wifi) {
    interrupt_wifi = false;

    q_corr = q_recenter * q;
    q_corr.toEuler(eulerAngles);

    OSCMessage msg("/q");
    msg.add(q_corr.scalar());
    msg.add(q_corr.x());
    msg.add(q_corr.y());
    msg.add(q_corr.z());
    udp.beginPacket(sendIP, SEND_PORT);
    msg.send(udp);
    udp.endPacket();

    OSCMessage msg_rpy("/euler_rpy");
    msg_rpy.add(eulerAngles.x());
    msg_rpy.add(eulerAngles.y());
    msg_rpy.add(eulerAngles.z());
    udp.beginPacket(sendIP, SEND_PORT);
    msg_rpy.send(udp);
    udp.endPacket();

    OSCMessage msg_acc("/acc");
    msg_acc.add(imu.m_accel.x());
    msg_acc.add(imu.m_accel.y());
    msg_acc.add(imu.m_accel.z());
    udp.beginPacket(sendIP, SEND_PORT);
    msg_acc.send(udp);
    udp.endPacket();


    OSCMessage msg_gyr("/gyr");
    msg_gyr.add(imu.m_gyro.x());
    msg_gyr.add(imu.m_gyro.y());
    msg_gyr.add(imu.m_gyro.z());
    udp.beginPacket(sendIP, SEND_PORT);
    msg_gyr.send(udp);
    udp.endPacket();


    OSCMessage msg_mag("/mag");
    msg_mag.add(imu.m_compass.x());
    msg_mag.add(imu.m_compass.y());
    msg_mag.add(imu.m_compass.z());
    udp.beginPacket(sendIP, SEND_PORT);
    msg_mag.send(udp);
    udp.endPacket();

    if(SEND_CALIBRATION){
      OSCMessage msg_acc_max("/acc_max");
      msg_acc_max.add(imu.m_accelMax.x());
      msg_acc_max.add(imu.m_accelMax.y());
      msg_acc_max.add(imu.m_accelMax.z());
      udp.beginPacket(sendIP, SEND_PORT);
      msg_acc_max.send(udp);
      udp.endPacket();

      OSCMessage msg_acc_min("/acc_min");
      msg_acc_min.add(imu.m_accelMin.x());
      msg_acc_min.add(imu.m_accelMin.y());
      msg_acc_min.add(imu.m_accelMin.z());
      udp.beginPacket(sendIP, SEND_PORT);
      msg_acc_min.send(udp);
      udp.endPacket();

      OSCMessage msg_mag_offset("/mag_offset");
      msg_mag_offset.add(imu.m_magOffset.x());
      msg_mag_offset.add(imu.m_magOffset.y());
      msg_mag_offset.add(imu.m_magOffset.z());
      udp.beginPacket(sendIP, SEND_PORT);
      msg_mag_offset.send(udp);
      udp.endPacket();

      OSCMessage msg_mag_scale("/mag_scale");
      msg_mag_scale.add(imu.m_magScale.x());
      msg_mag_scale.add(imu.m_magScale.y());
      msg_mag_scale.add(imu.m_magScale.z());
      udp.beginPacket(sendIP, SEND_PORT);
      msg_mag_scale.send(udp);
      udp.endPacket();
    }

    if (SERIAL_PROC) {
      Serial.print("Accel: ");
      Serial.print(imu.m_accel.x(), 6);   Serial.print(" ");
      Serial.print(imu.m_accel.y(), 6); Serial.print(" ");
      Serial.println(imu.m_accel.z(), 6);

      Serial.print("Gyro: ");
      Serial.print(imu.m_gyro.x() * RTMATH_DEGREE_TO_RAD, 6);   Serial.print(" ");
      Serial.print(imu.m_gyro.y() * RTMATH_DEGREE_TO_RAD, 6); Serial.print(" ");
      Serial.println(imu.m_gyro.z() * RTMATH_DEGREE_TO_RAD, 6);

      Serial.print("Compass: ");
      Serial.print(imu.m_compass.x(), 6);   Serial.print(" ");
      Serial.print(imu.m_compass.y(), 6); Serial.print(" ");
      Serial.println(imu.m_compass.z(), 6);

      Serial.print("Quaternion: ");
      Serial.print(q.scalar(), 6);   Serial.print(" ");
      Serial.print(q.x(), 6); Serial.print(" ");
      Serial.print(q.y(), 6); Serial.print(" ");
      Serial.println(q.z(), 6);
    }
  }

  yield();
}
