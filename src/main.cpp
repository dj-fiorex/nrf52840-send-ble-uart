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
#define BNO055_SAMPLERATE_DELAY_MS (10000)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

static imu::Vector<3> euler;

#define CFG_DEBUG 1
MyBleDevice myBleDevice;

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
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

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}

void setup()
{
  Serial.begin(115200);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while (!Serial)
    yield();
  delay(5000);
#endif

  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  // Initialize the library

  myBleDevice = MyBleDevice();
  myBleDevice.setup();
  myBleDevice.setConnectedCallback(connect_callback);
  myBleDevice.setDisconnectedCallback(disconnect_callback);

  myBleDevice.begin();

  // Set up and start advertising
  myBleDevice.startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
}

byte str[18];
double doubleArray[4];
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
  euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Serial.println(sizeof(euler.x()));

  doubleArray[0] = euler.x();

  doubleArray[1] = euler.y();
  doubleArray[2] = euler.z();

  byte *doublePtr = (byte *)doubleArray;
  Serial.println(sizeof(float));
  Serial.println(sizeof(double));

  // Add a delimiter
  doublePtr[24] = 0x3E;

  myBleDevice.write(doublePtr, 25);

  // Send doublePtr to BLE

  // for (byte i = 0; i < sizeof(doubleArray); i++)
  //{
  //   myBleDevice.write(doublePtr[i]);
  // }

  // str[0] = euler.x();
  // str[8] = 0x3E;
  // str[9] = euler.y();
  // str[17] = 0x3E;
  // myBleDevice.write(str, sizeof(str));
  // for (byte i = 0; i < sizeof(str); i++)
  // {
  //   myBleDevice.write(str, sizeof(str));
  // }

  // sprintf(str, "%f", euler.y());
  // myBleDevice.write(str);
  //  myBleDevice.write(euler.y());
  //  myBleDevice.write(euler.z());

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  /*
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
  */

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

void loop()
{
  // Forward data from HW Serial to BLEUART
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    myBleDevice.write(buf, count);
  }

  // Forward from BLEUART to HW Serial
  while (myBleDevice.available())
  {
    uint8_t ch;
    ch = (uint8_t)myBleDevice.read();
    Serial.write(ch);
  }

  readSensor();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
