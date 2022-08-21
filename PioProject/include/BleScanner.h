#ifndef BLE_SCANNER_H
#define BLE_SCANNER_H

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <array>
#include <vector>
#include <algorithm>    // std::sort
#include <queue>
#include <utility>
#include <mutex>

// IDF headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp32-hal-bt.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

// Tweaked SDK configuration
#include "sdkconfig.h"
#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BLEEddystoneURL.h"
#include "BLEEddystoneTLM.h"
#include "BLEBeacon.h"

struct iBeaconDeviceId_t {
    iBeaconDeviceId_t(const BLEUUID uuid, std::string Name) : uuid(uuid), name(std::move(Name)), power(0), lastSentRssi(0), confidence(0), state(0), last_update_millis(0)
    {

    }

    bool isVirgin () const {
        return !filled_once;
    }

    void reset() {
        power = 0;
        confidence = 0;
        state = 0;
        memset(rssi_array, 0, sizeof(rssi_array));
        filled_once=false;
    }

    void addRSSI(int rssi) {
        if(!filled_once && rssi_idx == 0) {
            memset(rssi_array, rssi, sizeof(rssi_array)); // Fill all with first value to get immediate 'find'
        }
        rssi_array[rssi_idx] = rssi;
        rssi_idx = (rssi_idx + 1)%rssi_array_size;
        if (!filled_once && rssi_idx == 0) {
            filled_once = true;
        }
    }

    int getFilteredRSSI() const {
        if(!filled_once) {
            return 0;
        }
        else {
            int copy[rssi_array_size];
            std::copy(std::begin(rssi_array), std::end(rssi_array), std::begin(copy));
            std::sort(copy, copy + rssi_array_size);
            return copy[2];
        }
    }

    BLEUUID      uuid;
    std::string  name;
    int          power;
    int          lastSentRssi;

    uint8_t confidence;
    uint8_t state;

    unsigned long last_update_millis;

    private:
        static const int rssi_array_size = 5;
        mutable int rssi_array[rssi_array_size];
        int rssi_idx = 0;
        bool filled_once = false;
};

class BleScanner : public BLEAdvertisedDeviceCallbacks
{
public:
    typedef std::function<void(const iBeaconDeviceId_t&)> DeviceUpdateCallbackFunction_t;

    BleScanner(Stream& serialStream) : mSerial(serialStream)  {
    }
    virtual ~BleScanner() {

    }

    void init();

    void setup();

    void loop();

    void stop();

    void setDeviceUpdateCallback(DeviceUpdateCallbackFunction_t callback);

    void setBeaconExpiration        (uint32_t             val     );
    void setMinTimeBetweenScans     (uint32_t             val     );
    void setPeriodicScanInterval    (uint32_t             val     );
    
    void startBluetoothScan();

    
    void addKnownDevice    (const BLEUUID uuid, const char* alias);
    void deleteKnownDevice (const BLEUUID uuid);
    void clearKnownDevices ();

    // BLE advertisement results:
    void onResult(BLEAdvertisedDevice advertisedDevice);
    // BLE scan done callback:
    void bleScanCompleted(BLEScanResults);
    
    void HandleBleAdvertisementResult(BLEAdvertisedDevice& bleAdvertisedDeviceResult);

private:

private:

    Stream& mSerial;

    std::vector<iBeaconDeviceId_t> iBeaconDevices;
    std::mutex iBeaconDevicesMutex;
    
    int scanTime = 4; //1; //In seconds
    BLEScan *pBLEScan = nullptr;
    bool bleScan_shouldStart = false;
    uint8_t scanContinueCount = 0;
    const uint8_t scanContinueWraparound = 4;
    const size_t maxBleProcessPerIteration = 3;

    std::queue<BLEAdvertisedDevice> bleAdvertisedDeviceResultQueue;
    DeviceUpdateCallbackFunction_t deviceUpdateCallback;

    // Scanner parameters from storage:
    uint8_t num_arrival_scans;
    uint8_t num_departure_scans;
    unsigned long scan_iter_interval;
    uint32_t beacon_expiration_seconds;
    uint32_t min_seconds_between_scans;
    uint32_t periodic_scan_interval;
};

#endif // BLE_SCANNER_H
