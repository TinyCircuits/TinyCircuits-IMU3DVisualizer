// Libraries for sensor and Wi-Fi communication
#include <SPI.h>
#include "RTIMUSettings.h"  // 9-axis sensor utility    
#include "RTIMU.h"          // 9-axis sensor utility
#include "RTFusionRTQF.h"   // 9-axis sensor utility
#include "BMA250.h"         // 3-axis sensor utility
#include <Wire.h>           // for I2C communication with sensors
#include <Wireling.h>       // For Wireling Interfacing
#include <WiFi101.h>        // this library is for the wifi connections


#define USB 1
#define WIFI 2

#define TINYZERO 1
#define ROBOTZERO 2
#define WIRELING9AXIS 3
#define WIRELING3AXIS 4


/****** EDIT THIS SECTION TO MATCH THE CONNECTION TO THE COMPUTER AND THE IMU DEVICE YOU ARE USING******/
int COMMUNICATION = USB;    // options: USB, WIFI
int BOARD = ROBOTZERO;      // options: TINYZERO, ROBOTZERO, WIRELING9AXIS, WIRELING3AXIS

/****** EDIT THIS SECTION TO MATCH YOUR WiFi INFO IF USING WIFI ******/
char ssid[] = "TinyCircuits";                     // your network SSID (name) that you see in network broswers
char wifi_password[] = "3336119997";      // your network password used to connect to the above SSID
char server_computer_ip[] = "192.168.1.107";  // local IP address of computer runnning Python script
uint16_t server_listen_port = 8090;           // port that client and python server use
WiFiClient client;                            // WiFi client to describe this arduino



// Used to handle LSM9DS1
RTIMU *imu;                         // the IMU object
RTFusionRTQF fusion;                // the fusion object
RTIMUSettings settings;             // the settings object
int DISPLAY_INTERVAL = 30;          // interval between pose displays
unsigned long lastDisplay;          // interval tracker between pose displays

// Used to handle BMA250
BMA250 three_axis;
float rollF;
float pitchF;


#ifndef ARDUINO_ARCH_SAMD
  #include <EEPROM.h>
#endif

// Make Serial Monitor compatible for all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif



void setup(void) {
  // set the serial BAUD rate
  SerialMonitorInterface.begin(115200);
  
  // start wire for sensor communication
  Wire.begin();

  // call functions for using either the
  // 3-axis BMA250 or 9-axis LSM9DS1 devices
  // depending on BOARD flag
  SetupSensorHardware();

  // USB is the default way to communicate to
  // the python script on the computer, don't
  // need to setup any hardware if that is being used
  if(COMMUNICATION != USB){
    SetupCommunicationHardware();
  }
}



void loop() {
  // send the polled 3-axis or 9-axis
  // over USB, BLE, or WIFI depending
  // on the flags set at the top
  switch(COMMUNICATION){
    case USB:   // hanlde USB data transfer
      HandleUSB();
    break;
    case WIFI:  // handle WiFi data transfer
      HandleWIFI();
    break;
  }
}



// connection to server is not tested
// in this function since it is run
// in the setup() function and BLE
// needs a couple of loops to connect
// to the python server
void SetupCommunicationHardware(){
  switch(COMMUNICATION){
    case WIFI:   // setup WiFi
      // set pins and wait for connection
      // for a certain amount of time
      WiFi.setPins(8, 2, A3, -1);
      WiFi.begin(ssid, wifi_password);
      int wifi_timeout_counter = 0;

      // try a couple of times before giving up on
      // connecting to the WIFI at the given ssid
      while (WiFi.status() != WL_CONNECTED){
        delay(100);
        wifi_timeout_counter += 1;
        if (wifi_timeout_counter >= 10){
          break;
        }
      }

      // wait a little longer and still try to
      // connect this client to the python server.
      delay(500);
      client.connect(server_computer_ip, server_listen_port);
    break;
  }
}



void SetupLSM9DS1(){
  int errcode;
  imu = RTIMU::createIMU(&settings);        // create the imu object

  SerialMonitorInterface.print("ArduinoIMU begin using device "); SerialMonitorInterface.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    SerialMonitorInterface.print("Failed to init IMU: "); SerialMonitorInterface.println(errcode);
  }

  // See line 69 of RTIMU.h for more info on compass calibaration 
  if (imu->getCalibrationValid())
    SerialMonitorInterface.println("Using compass calibration");
  else
    SerialMonitorInterface.println("No valid compass calibration data");

  lastDisplay = millis();

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}



// use appropriate functions to enable
// the on-board IMU devices depending
// on the set device flags set at the
// top of this file
void SetupSensorHardware(){
  Wire.begin();
  switch(BOARD){
    case TINYZERO:        // 3-axis
      three_axis.begin(BMA250_range_2g, BMA250_update_time_32ms);
      
    break;
    case ROBOTZERO:       // 9-axis
      SetupLSM9DS1();
      delay(100);

    break;
    case WIRELING9AXIS:   // 9-axis
      // Initialize Wireling
      Wireling.begin();
      Wireling.selectPort(0);
      delay(100);
      SetupLSM9DS1();

    break;
    case WIRELING3AXIS:   // 3 axis
      // Initialize Wireling
      Wireling.begin();
      Wireling.selectPort(0);
      delay(100);
      three_axis.begin(BMA250_range_2g, BMA250_update_time_32ms);
      
    break;
  }
  delay(100);
}



String ReadLSM9DS1(){
  unsigned long now = millis();
  unsigned long delta;

  if (imu->IMURead()) {     // get the latest data if ready 
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;

      RTQuaternion sample = fusion.getFusionQPose();
      
      return String(BOARD) + " " + String(sample.data(0)) + " " + String(sample.data(1)) + " " + String(sample.data(2)) + " " + String(sample.data(3));
    }
  }
  return "";
}



// Reads BMA250 3-axis data to the
// global acceleration variables
String ReadBMA250(){
  // Get new acccelerometer data
  three_axis.read();

  // Store the data. Note that values
  // of -1, -1, and -1 will be output
  // if no sensor found
  float aX = three_axis.X;
  float aY = three_axis.Y;
  float aZ = three_axis.Z;

  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  float roll = atan(aY / sqrt(pow(aX, 2) + pow(aZ, 2))) * 180 / PI;
  float pitch = atan(-1 * aX / sqrt(pow(aY, 2) + pow(aZ, 2))) * 180 / PI;

  // low pass filter the results to stop jitter
  rollF = 0.9 * rollF + 0.1 * roll;
  pitchF = 0.9 * pitchF + 0.1 * pitch;

  // Set to XX as per BMA250_update_time_XXms
  // in the setup, otherwise there is no output
  delay(32);

  // Speed is not of the essence here, to keep
  // this easy to read convert and combine
  // data that is sent to python is String.
  // Send board every time so can be swapped
  // without script restart
  return String(BOARD) + " " + String(rollF) + " " + String (pitchF);
}



// handle sending data over USB to python
void HandleUSB(){
  switch(BOARD){
    case TINYZERO:        // 3-axis IMU
      SerialMonitorInterface.println(ReadBMA250());
    break;
    case ROBOTZERO:       // 9-axis IMU
      {
        String output = ReadLSM9DS1();
        if(output != ""){
          SerialMonitorInterface.println(output);
        }
      }
    break;
    case WIRELING9AXIS:   // 9-axis IMU
      {
        String output = ReadLSM9DS1();
        if(output != ""){
          SerialMonitorInterface.println(output);
        }
      }
    break;
    case WIRELING3AXIS:   // 3-axis IMU
      SerialMonitorInterface.println(ReadBMA250());
    break;
  }
}



// handle sending data over WIFI to python
void HandleWIFI(){
  switch(BOARD){
    case TINYZERO:        // 3-axis IMU
      client.println(ReadBMA250());
    break;
    case ROBOTZERO:       // 9-axis IMU
      {
        String output = ReadLSM9DS1();
        if(output != ""){
          client.println(output);
        }
      }
    break;
    case WIRELING9AXIS:   // 9-axis IMU
      {
        String output = ReadLSM9DS1();
        if(output != ""){
          client.println(output);
        }
      }
    break;
    case WIRELING3AXIS:   // 3-axis IMU
      client.println(ReadBMA250());
    break;
  }  
}
