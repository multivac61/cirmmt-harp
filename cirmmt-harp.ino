#include <Wire.h>
#include "lib/MPU9250.h"
#include "Adafruit_DRV2605.h"

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>

#include "lib/OSCMessage.h"

extern "C" {
#include "user_interface.h"
}

#define SERIAL_DEBUG true // Print initialization messages over serial
#define SERIAL_PROC false // Print data messages over serial
#define WAIT_WIFI 30      // UDP send interval
#define WAIT_FILTER 10    // quaternion filter process interval

// Initialization of software interrupt functions
os_timer_t wifiTimer;
os_timer_t filterTimer;

bool interrupt_wifi;
bool interrupt_filter;

void timerCallback(void *pArg) {
  interrupt_wifi = true;
}

void filterCallback(void *pArg) {
  interrupt_filter = true;
}

void user_init(void) {
  os_timer_setfn(&wifiTimer, timerCallback, NULL);
  os_timer_arm(&wifiTimer, WAIT_WIFI, true);

  os_timer_setfn(&filterTimer, filterCallback, NULL);
  os_timer_arm(&filterTimer, WAIT_FILTER, true);
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
IPAddress home(192, 168, 1, 101); 
IPAddress broadcast(192, 168, 1, 255);
IPAddress mask(255, 255, 255, 0);

// Static IP of receiving client
IPAddress sendIP(192, 168, 1, 42);

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

  imu.selfTest();
  imu.selfTestReport();

  Serial.println("MPU9250 calibration");
  imu.calibrate();

  Serial.println("MPU9250 magnetometer calibration");
  imu.calibrate_mag();

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
  interrupt_filter = false;
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
  }

  if (interrupt_filter) {
    interrupt_filter = false;
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
    msg_acc.add(imu.acc[0]);
    msg_acc.add(imu.acc[1]);
    msg_acc.add(imu.acc[2]);
    udp.beginPacket(sendIP, SEND_PORT);
    msg_acc.send(udp);
    udp.endPacket();

    OSCMessage msg_gyr("/gyr");
    msg_gyr.add(imu.gyr[0]);
    msg_gyr.add(imu.gyr[1]);
    msg_gyr.add(imu.gyr[2]);
    udp.beginPacket(sendIP, SEND_PORT);
    msg_gyr.send(udp);
    udp.endPacket();

    OSCMessage msg_mag("/mag");
    msg_mag.add(imu.mag[0]);
    msg_mag.add(imu.mag[1]);
    msg_mag.add(imu.mag[2]);
    udp.beginPacket(sendIP, SEND_PORT);
    msg_mag.send(udp);
    udp.endPacket();

    if (SERIAL_PROC) {
      Serial.print("Orientation: ");
      Serial.print(yaw);   Serial.print(" ");
      Serial.print(pitch); Serial.print(" ");
      Serial.println(roll);
    }
  }

  yield();
  delay(50); // added this wait function, have to research if it does any good
}
