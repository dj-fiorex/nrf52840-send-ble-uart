#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
class MyBleDevice
{                  // The class
public:            // Access specifier
    MyBleDevice(); // Constructor

    void setup(void);
    void begin(void);
    void startAdv(void);
    void setConnectedCallback(ble_connect_callback_t fp);
    void setDisconnectedCallback(ble_disconnect_callback_t fp);
    size_t write(const uint8_t *content, size_t len);
    size_t write(uint8_t b);
    size_t write(const char *str);
    int read(void);
    int available(void);

private:
    // BLE Service
    BLEDfu bledfu;   // OTA DFU service
    BLEDis bledis;   // device information
    BLEUart bleuart; // uart over ble
    BLEBas blebas;   // battery
};