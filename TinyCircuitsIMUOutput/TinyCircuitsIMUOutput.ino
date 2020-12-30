// Libraries for sensor and Wi-Fi communication
#include <SPI.h>
#include "lib_aci.h"        // 9-axis LSM9DS1
#include "aci_setup.h"      // 9-axis LSM9DS1
#include "BMA250.h"         // for interfacing with the 3-axis accel. sensor
#include <Wire.h>           // for I2C communication with sensors
#include <Wireling.h>       // For Wireling Interfacing
#include <WiFi101.h>        // this library is for the wifi connection


// This object is for the USB serial connection
#if defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#else
  #define SerialMonitorInterface Serial
#endif

#define LSM9DS1_ACCEL_SCALE_4G 0.000122   // see LSM9DS1.ino lines 387 ~ 407
#define LSM9DS1_GYRO_SCALE_500DPS 0.0175  // see LSM9DS1.ino lines 387 ~ 407
#define LSM9DS1_MAG_SCALE 0.014           // see LSM9DS1.ino lines 387 ~ 407

#define USB 1
#define WIFI 2

#define TINYZERO 1
#define ROBOTZERO 2
#define WIRELING9AXIS 3
#define WIRELING3AXIS 4


/****** EDIT THIS SECTION TO MATCH THE CONNECTION TO THE COMPUTER AND THE IMU DEVICE YOU ARE USING******/
int COMMUNICATION = USB;    // options: USB, WIFI
int BOARD = WIRELING3AXIS;  // options: TINYZERO, ROBOTZERO, WIRELING9AXIS, WIRELING3AXIS


/****** EDIT THIS SECTION TO MATCH YOUR WiFi INFO IF USING WIFI ******/
char ssid[] = "TestWiFi";                     // your network SSID (name) that you see in network broswers
char wifi_password[] = "securepassword";      // your network password used to connect to the above SSID
char server_computer_ip[] = "192.168.0.118";  // local IP address of computer runnning Python script
uint16_t server_listen_port = 8090;           // port that client and python server use
WiFiClient client;                            // WiFi client to describe this arduino


// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System), when using LSM9DS1
float gyro_meas_error = PI * (0.0f / 180.0f);         // gyroscope measurement error in rads/s (start at 40 deg/s)
float gyro_meas_drift = PI * (500.0f  / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * gyro_meas_error;     // compute beta
float zeta = sqrt(3.0f / 4.0f) * gyro_meas_drift;     // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float accelerometer_bias[3] = {0, 0, 0};              // bias correction for accelerometer
float gyroscope_bias[3] = {0, 0, 0};                  // bias correction for gyroscope
float magnetometer_bias[3] = {0, 0, 0};               // bias correction for magnetometer
float deltat = 0.0f, sum = 0.0f;                      // integration interval
uint32_t last_update = 0, first_update = 0, now = 0;  // used to calculate integration interval
uint32_t sum_count = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                // vector to hold rotation quaternion

// Accelerometer, gyroscope, magnetometer, and
// temperature reading variables from LSM9DS1
int aX = -1, aY = -1, aZ = -1, gX = -1, gY = -1, gZ = -1, mX = -1, mY = -1, mZ = -1, tempF = -1;
float roll, pitch, yaw, rollF, pitchF, yawF;

// Used to handle BMA250, accelerometer readings
// are stored internally, unlike LSM9DS1 above
BMA250 three_axis;


void setup(void) {
  // set the serial BAUD rate
  SerialMonitorInterface.begin(115200);

  // start wire for sensor communication
  Wire.begin();

  // USB is the default way to communicate to
  // the python script on the computer, don't
  // need to setup any hardware if that is being used
  if(COMMUNICATION != USB){
    SetupCommunicationHardware();
  }

  // call functions for using either the
  // 3-axis BMA250 or 9-axis LSM9DS1 devices
  // depending on BOARD flag
  SetupSensorHardware();
}



void loop() {
  CheckCommunicationConnection();

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

      // try a couple of times before giving on
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


// takes over main loop by looping
// forever and blinking on-board LED
// to indicate that an error occured
// and to look at throubleshooting tips
// online for current hardware setup
void ReportErrorVisually(){
  while(true){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
  }
}


// checks that this client communicating
// tot he serrver python script and if
// not call function that blinks LED
// forever
void CheckCommunicationConnection(){
  switch(COMMUNICATION){
    case WIFI:  // test WiFi
      if(WiFi.status() != WL_CONNECTED){
        //ReportErrorVisually();
      }
    break;
  }
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
      IMUInit();
      delay(100);

      // fill the bias vectors
      CalculateSensorBiases(); 
    case WIRELING9AXIS:  // 9-axis
      // Initialize Wireling
      Wireling.begin();
      Wireling.selectPort(0); //9-Axis Sensor Port, may differ for you
      delay(100);
      IMUInit();
      delay(100);
      
      // fill the bias vectors
      CalculateSensorBiases();
    case WIRELING3AXIS: // 3 axis
      // Initialize Wireling
      Wireling.begin();
      Wireling.selectPort(0); //9-Axis Sensor Port, may differ for you
      delay(100);
      IMUInit();
      delay(100);
    
      three_axis.begin(BMA250_range_2g, BMA250_update_time_32ms); 
    break;
  }
  delay(100);
}


// see https://github.com/kriswiner/LSM9DS1/blob/065c252be9e274d9d03ac54c1e3fb0d54e48701c/LSM9DS1_MS5611_BasicAHRS_t3.ino#L762
// fills bias vectors with averged data on each axis from each IMU device when using LSM9DS1 only
void CalculateSensorBiases(){

  // Buffer to store multiple readings, on each axis, in 32 bit signed int
  int32_t tempAccelBias[3] = {0,0,0};
  int32_t tempGyroBias[3] = {0,0,0};
  int16_t tempMagBias[3] = {0,0,0};
  int16_t magMax[3] = {0, 0, 0}, magMin[3] = {0, 0, 0};

  // Number of samples to take at rest
  byte N = 32;

  // Take N samples of data from each sensor
  for(int ai=0; ai < N; ai++){
    IMURead();
    tempAccelBias[0] += (int32_t)aX;
    tempAccelBias[1] += (int32_t)aY;
    tempAccelBias[2] += (int32_t)aZ;
    
    tempGyroBias[0] += (int32_t)gX;
    tempGyroBias[1] += (int32_t)gY;
    tempGyroBias[2] += (int32_t)gZ;

    tempMagBias[0] = (int16_t)mX;
    tempMagBias[1] = (int16_t)mY;
    tempMagBias[2] = (int16_t)mZ;

    for (int mi = 0; mi < 3; mi++) {
      if(tempMagBias[mi] > magMax[mi]) magMax[mi] = tempMagBias[mi];
      if(tempMagBias[mi] < magMin[mi]) magMin[mi] = tempMagBias[mi];
    }
    delay(32);
  }

  // Calculate the avg accel from resting data
  accelerometer_bias[0] = tempAccelBias[0] / N;  // X
  accelerometer_bias[1] = tempAccelBias[1] / N;  // Y
  accelerometer_bias[2] = tempAccelBias[2] / N;  // Z

  // Remove gravity from the z-axis accelerometer bias calculation
  if(accelerometer_bias[2] > 0L) {
    accelerometer_bias[2] -= (int32_t) (1.0/LSM9DS1_ACCEL_SCALE_4G);
  }
  else {
    accelerometer_bias[2] += (int32_t) (1.0/LSM9DS1_ACCEL_SCALE_4G);
  }

  accelerometer_bias[0] = (float)accelerometer_bias[0]*LSM9DS1_ACCEL_SCALE_4G;  // Properly scale the data to get g
  accelerometer_bias[1] = (float)accelerometer_bias[1]*LSM9DS1_ACCEL_SCALE_4G;
  accelerometer_bias[2] = (float)accelerometer_bias[2]*LSM9DS1_ACCEL_SCALE_4G;

  gyroscope_bias[0] = tempGyroBias[0] / N;  // X
  gyroscope_bias[1] = tempGyroBias[1] / N;  // Y
  gyroscope_bias[2] = tempGyroBias[2] / N;  // Z

  gyroscope_bias[0] = (float)gyroscope_bias[0]*LSM9DS1_GYRO_SCALE_500DPS;  // Properly scale the data to get deg/s
  gyroscope_bias[1] = (float)gyroscope_bias[1]*LSM9DS1_GYRO_SCALE_500DPS;
  gyroscope_bias[2] = (float)gyroscope_bias[2]*LSM9DS1_GYRO_SCALE_500DPS;

  tempMagBias[0] = (magMax[0] + magMin[0])/2;  // X
  tempMagBias[1] = (magMax[1] + magMin[1])/2;  // Y
  tempMagBias[2] = (magMax[2] + magMin[2])/2;  // Z
  
  magnetometer_bias[0] = (float) tempMagBias[0]*LSM9DS1_MAG_SCALE;  // save mag biases in G for main program
  magnetometer_bias[1] = (float) tempMagBias[1]*LSM9DS1_MAG_SCALE;   
  magnetometer_bias[2] = (float) tempMagBias[2]*LSM9DS1_MAG_SCALE;   
}


// Reads BMA250 3-axis data to the
// global acceleration variables
String ReadBMA250(){
  // Get new acccelerometer data
  three_axis.read();

  // Store the data. Note that values
  // of -1, -1, and -1 will be output
  // if no sensor found
  aX = three_axis.X;
  aY = three_axis.Y;
  aZ = three_axis.Z;

  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  roll = atan(aY / sqrt(pow(aX, 2) + pow(aZ, 2))) * 180 / PI;
  pitch = atan(-1 * aX / sqrt(pow(aY, 2) + pow(aZ, 2))) * 180 / PI;

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
  return String(BOARD) + " " + String(-rollF) + " " + String (-pitchF);
}


// Read LSM9DS1 data to the global
// acceleration, magnetometer, and
// gyroscope 3-axis (each) variables
String ReadLSM9DS1(){
  // Read and print 9-Axis Accelerometer and Temperature data 
  //(gyro and magentometer data can also be printed using the gX, gY, gZ, mX, mY, and mZ variables)
  // (called from LSM9DS1.ino)
  IMURead();

  float aXf, aYf, aZf, gXf, gYf, gZf, mXf, mYf, mZf; // variables to hold latest sensor data values

  // Only have the raw data in int, need to convert to float
  // by using resolutions and scales
  // Now we'll calculate the accleration value into actual g's
  // see https://github.com/kriswiner/LSM9DS1/blob/065c252be9e274d9d03ac54c1e3fb0d54e48701c/LSM9DS1_MS5611_BasicAHRS_t3.ino#L420
  aXf = (float)aX*LSM9DS1_ACCEL_SCALE_4G - accelerometer_bias[0];  // get actual g value, this depends on scale being set
  aYf = (float)aY*LSM9DS1_ACCEL_SCALE_4G - accelerometer_bias[1];
  aZf = (float)aZ*LSM9DS1_ACCEL_SCALE_4G - accelerometer_bias[2];

  // Calculate the gyro value into actual degrees per second
  gXf = (float)gX*LSM9DS1_GYRO_SCALE_500DPS - gyroscope_bias[0];  // get actual gyro value, this depends on scale being set
  gYf = (float)gY*LSM9DS1_GYRO_SCALE_500DPS - gyroscope_bias[1];
  gZf = (float)gZ*LSM9DS1_GYRO_SCALE_500DPS - gyroscope_bias[2];

  mXf = (float)mX*LSM9DS1_MAG_SCALE - magnetometer_bias[0];  // get actual magnetometer value, this depends on scale being set
  mYf = (float)mY*LSM9DS1_MAG_SCALE - magnetometer_bias[1];
  mZf = (float)mZ*LSM9DS1_MAG_SCALE - magnetometer_bias[2];

  now = micros();
  deltat = ((now - last_update)/1000000.0f); // set integration time by time elapsed since last filter update
  last_update = now;

  sum += deltat; // sum for averaging filter update rate
  sum_count++;

  // See this link about why signs are flipp negative: https://github.com/kriswiner/LSM9DS0/issues/10#issuecomment-261585795
  MadgwickQuaternionUpdate(aXf, aYf, aZf, -gXf*PI/180.0f, -gYf*PI/180.0f, -gZf*PI/180.0f,  -mXf,  mYf, mZf);
  yaw   = (atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])) * (180.0f / PI);
  pitch = (-asin(2.0f * (q[1] * q[3] - q[0] * q[2]))) * (180.0f / PI);
  roll  = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])) * (180.0f / PI);

  // low pass filter the results to stop jitter
  rollF = 0.9 * rollF + 0.1 * roll;
  pitchF = 0.9 * pitchF + 0.1 * pitch;
  yawF = 0.9 * yawF + 0.1 * yaw;
  delay(32);

  // Speed is not of the essence here, to keep
  // this easy to read convert and combine
  // data that is sent to python is String
  // Send board every time so can be swapped
  // without script restart
  return String(BOARD) + " " + String(q[0]) + " " + String(q[1]) + " " + String(q[2]) + " " + String(q[3]);
}


// handle sending data over USB to python
void HandleUSB(){
  switch(BOARD){
    case TINYZERO:        // 3-axis IMU
      SerialMonitorInterface.println(ReadBMA250());
    break;
    case ROBOTZERO:       // 9-axis IMU
      SerialMonitorInterface.println(ReadLSM9DS1());
    break;
    case WIRELING9AXIS:   // 9-axis IMU
      SerialMonitorInterface.println(ReadLSM9DS1());
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
      client.println(ReadLSM9DS1());
    break;
    case WIRELING9AXIS:   // 9-axis IMU
      client.println(ReadLSM9DS1());
    break;
    case WIRELING3AXIS:   // 3-axis IMU
      SerialMonitorInterface.println(ReadBMA250());
    break;
  }  
}
