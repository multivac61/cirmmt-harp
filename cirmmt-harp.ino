#include <Wire.h>
#include "lib/MPU9250.h"
#include "Adafruit_DRV2605.h"

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>

#include "lib/OSCMessage.h"

extern "C" {
#include "user_interface.h"
}

#define SERIAL_DEBUG true      // Print initialization messages over serial
#define SERIAL_PROC false      // Print data messages over serial
#define SEND_CALIBRATION false // Send calibration constants over UDP
#define WAIT_WIFI 5           // UDP send interval

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

// Vector to hold quaternion data
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

MPU9250 imu(MPU9250_DEFAULT_ADDRESS);

// Wifi configuration //
WiFiUDP udp;

// Define the SSID and password of network
const char* ssid = "";
const char* password = "";
#define LISTEN_PORT 5001  // Port for incoming OSC messages
#define SEND_PORT 58039   // Port for outgoing OSC messages

// Static IP address of module
IPAddress home(192, 168, 10, 44); 
IPAddress broadcast(192, 168, 1, 255);
IPAddress mask(255, 255, 255, 0);

// Static IP of receiving client
IPAddress sendIP(192, 168, 10, 103);

// Global variable for haptic motor
Adafruit_DRV2605 drv;

// Onboard RGB diode pins
int redPin = 5;
int greenPin = 4;
int bluePin = 15;

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
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin(12, 13);

  Serial.begin(38400);
  delay(500);

  while (!imu.testConnection()) {
    if (SERIAL_DEBUG) {
      Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(imu.getDeviceID(), HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
      Serial.println(". Could not connect to MPU9250");
    }
    delay(500);
  }

  if (SERIAL_DEBUG) Serial.println("MPU9250 is online...");

  imu.init();

  if (SERIAL_DEBUG) Serial.println("MPU9250 initialized for active data mode....");

  while (!imu.testMagConnection()) {
    if (SERIAL_DEBUG) {
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(imu.getMagID(), HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
      Serial.println(". Could not connect to AK8963");
    }
    delay(500);
  }

  if (SERIAL_DEBUG) {
    Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(imu.magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(imu.magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(imu.magCalibration[2], 2);
  }

  // Setup the wifi connection
  WiFi.disconnect();
  delay(200);
  WiFi.begin(ssid, password);         // Connect to Wifi network
  WiFi.config(home, broadcast, mask); // Obtain a static IP address
  delay(200);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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
    Serial.println("Setup complete!");
  }

  drv.begin(12, 13);
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); 

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Software interrupts
  interrupt_wifi = false;
  user_init();
}


void loop() {
  int size = udp.parsePacket();
  if(size > 0){
    OSCMessage incoming_osc;
    while(size--){
      incoming_osc.fill(udp.read());
    }
    if(SERIAL_PROC){
      printMessage(incoming_osc);
    }
    
    char address[64];
    int len = incoming_osc.getAddress(address, 0);

    if(address[1] == 'd'){ // driver
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

    } else if(address[1] == 'l'){ // LED 
      int r = incoming_osc.getInt(0);
      int g = incoming_osc.getInt(1);
      int b = incoming_osc.getInt(2);
      setColor(r, g, b);
    }
  }

  if (imu.isInterrupted()) {
    imu.readAllData();
    imu.updateFilter(q);
  }

  if (interrupt_wifi) {
    interrupt_wifi = false;

    float yaw, pitch, roll;
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    roll  *= 180.0f / PI;

    OSCMessage msg("/q");
    msg.add(q[0]);
    msg.add(q[1]);
    msg.add(q[2]);
    msg.add(q[3]);
    udp.beginPacket(sendIP, SEND_PORT);
    msg.send(udp);
    udp.endPacket();

    OSCMessage msg_rpy("/rpy");
    msg_rpy.add(roll);
    msg_rpy.add(pitch);
    msg_rpy.add(yaw);
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
      Serial.print(imu.m_gyro.x(), 6);   Serial.print(" ");
      Serial.print(imu.m_gyro.y(), 6); Serial.print(" ");
      Serial.println(imu.m_gyro.z(), 6);

      Serial.print("Compass: ");
      Serial.print(imu.m_compass.x(), 6);   Serial.print(" ");
      Serial.print(imu.m_compass.y(), 6); Serial.print(" ");
      Serial.println(imu.m_compass.z(), 6);

      Serial.print("Quaternion: ");
      Serial.print(q[0], 6);   Serial.print(" ");
      Serial.print(q[1], 6); Serial.print(" ");
      Serial.print(q[2], 6); Serial.print(" ");
      Serial.println(q[3], 6);
    }
  }

  yield();
  delay(50); // added this wait function, have to research if it does any good
}
