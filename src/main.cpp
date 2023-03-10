#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "MyBleDevice.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

static imu::Vector<3> accelerometer;
static imu::Vector<3> magnetometer;
static imu::Vector<3> gyroscope;
static imu::Vector<3> euler;
static imu::Vector<3> linearaccel;
static imu::Vector<3> gravity;

// #define DEBUG_ENV 1
MyBleDevice myBleDevice;

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

#if DEBUG_ENV

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));
  auto mtu = connection->getMtu();

  Serial.println(mtu);
  Serial.print("Connected to ");
  Serial.println(central_name);

  // request PHY changed to 2MB
  Serial.println("Request to change PHY");
#endif

  connection->requestPHY();

// request to update data length
#if DEBUG_ENV

  Serial.println("Request to change Data Length");
#endif

  connection->requestDataLengthUpdate();

// request mtu exchange
#if DEBUG_ENV
  Serial.println("Request to change MTU");
#endif
  connection->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  // conn->requestConnectionParameter(6); // in unit of 1.25

  // delay a bit for all the request to complete
  delay(1000);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

#if DEBUG_ENV
  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
#endif
}

// #define VBATPIN A6
#define BAT_HIGH_CHARGE 13          // HIGH for 50mA, LOW for 100mA
#define BAT_CHARGE_STATE 23         // LOW for charging, HIGH not charging
#define VBAT_PER_LBS (0.003515625F) // 3.6 reference and 10 bit resolution

void setup()
{
#if DEBUG_ENV
  Serial.begin(115200);

  // Blocking wait for connection when debug mode is enabled via IDE
  while (!Serial)
    yield();
  delay(5000);

  // float measuredvbat = analogRead(VBATPIN);
  // measuredvbat *= 2;    // we divided by 2, so multiply back
  ////measuredvbat *= 3.6;  // Multiply by 3.6V, our reference voltage
  // measuredvbat /= 1024; // convert to voltage
  // Serial.print("VBat: ");
  // Serial.println(measuredvbat);

  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  pinMode(VBAT_ENABLE, OUTPUT);
  delay(1000);
  Serial.println("Bluefruit52 BLEUART Example");
  pinMode(BAT_CHARGE_STATE, INPUT);
  delay(1000);
  Serial.println("Bluefruit52 BLEUART Example");
  digitalWrite(BAT_HIGH_CHARGE, HIGH); // charge with 100mA
  delay(1000);
  Serial.println("Bluefruit52 BLEUART Example");
  digitalWrite(VBAT_ENABLE, LOW);
  Serial.println("Bluefruit52 BLEUART Example");
#endif

  delay(5000);
  /* Initialise the sensor */
  if (!bno.begin())
  {
#if DEBUG_ENV
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
#endif

    while (1)
      ;
  }
  delay(1000);

#if DEBUG_ENV

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
#endif

  bno.setExtCrystalUse(true);
#if DEBUG_ENV

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
#endif

  // Initialize the library

  myBleDevice = MyBleDevice();
  myBleDevice.setup();
  myBleDevice.setConnectedCallback(connect_callback);
  myBleDevice.setDisconnectedCallback(disconnect_callback);

  myBleDevice.begin();

  // Set up and start advertising
  myBleDevice.startAdv();
#if DEBUG_ENV

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
#endif
}

float doubleArray[19];
void readSensor()
{
  // Read sensor data
  // .
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

#if DEBUG_ENV
  // Serial.print(accelerometer.x());
  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_PER_LBS;
  float vBat = adcVoltage * (1510.0 / 508.0);
  Serial.println(vBat);
  delay(1000);
#endif

  doubleArray[0] = (float)accelerometer.x();
  doubleArray[1] = (float)accelerometer.y();
  doubleArray[2] = (float)accelerometer.z();

  doubleArray[3] = (float)magnetometer.x();
  doubleArray[4] = (float)magnetometer.y();
  doubleArray[5] = (float)magnetometer.z();

  doubleArray[6] = (float)gyroscope.x();
  doubleArray[7] = (float)gyroscope.y();
  doubleArray[8] = (float)gyroscope.z();

  doubleArray[9] = (float)euler.x();
  doubleArray[10] = (float)euler.y();
  doubleArray[11] = (float)euler.z();

  doubleArray[12] = (float)linearaccel.x();
  doubleArray[13] = (float)linearaccel.y();
  doubleArray[14] = (float)linearaccel.z();

  doubleArray[15] = (float)gravity.x();
  doubleArray[16] = (float)gravity.y();
  doubleArray[17] = (float)gravity.z();

  byte *doublePtr = (byte *)doubleArray;

  // Add a delimiter
  doublePtr[72] = 0xAA;
  doublePtr[73] = 0x3A;

  myBleDevice.write(doublePtr, 74);
}

void loop()
{
#if DEBUG_ENV
  // Forward from BLEUART to HW Serial
  while (myBleDevice.available())
  {
    uint8_t ch;
    ch = (uint8_t)myBleDevice.read();
    Serial.write(ch);
  }
#endif

  readSensor();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void displayCalibration()
{
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}

void displayQuaternion()
{
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
}
