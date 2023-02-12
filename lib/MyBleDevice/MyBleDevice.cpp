#include "MyBleDevice.h"
#include <functional>

MyBleDevice::MyBleDevice()
{
}

void MyBleDevice::setup(void)
{

    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behavior, but provided
    // here in case you want to control this LED manually via PIN 19
    Bluefruit.autoConnLed(true);

    // Config the peripheral connection with maximum bandwidth
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    Bluefruit.begin();
    Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
    // Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
}

void MyBleDevice::begin(void)
{

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Start BLE Battery Service
    blebas.begin();
    blebas.write(100);
}

void MyBleDevice::startAdv(void)
{

    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void MyBleDevice::setConnectedCallback(ble_connect_callback_t fp)
{
    Bluefruit.Periph.setConnectCallback(fp);
}
void MyBleDevice::setDisconnectedCallback(ble_disconnect_callback_t fp)
{
    Bluefruit.Periph.setDisconnectCallback(fp);
}

size_t MyBleDevice::write(const char *str)
{
    return bleuart.write(str);
}

size_t MyBleDevice::write(const uint8_t *content, size_t len)
{

    return bleuart.write(content, len);
}

size_t MyBleDevice::write(uint8_t b)
{
    return bleuart.write(b);
}

int MyBleDevice::read(void)
{
    return (uint8_t)bleuart.read();
}

int MyBleDevice::available(void)
{
    return bleuart.available();
}
