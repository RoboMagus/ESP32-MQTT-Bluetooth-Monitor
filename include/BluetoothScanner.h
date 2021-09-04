#ifndef BLUETOOTH_SCANNER_H
#define BLUETOOTH_SCANNER_H

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <array>
#include <vector>
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

// monitor/setup/ADD STATIC DEVICE
// monitor/setup/add known device
// monitor/setup/delete known device

char *bda2str(const esp_bd_addr_t bda, char *str, size_t size);
bool  str2bda(const char* str, esp_bd_addr_t& mac);

enum class ScanType{
    None,
    Arrival,
    Departure,
    Either
};

class BluetoothScanner : public BLEAdvertisedDeviceCallbacks
{
public:
    BluetoothScanner() {
    }
    virtual ~BluetoothScanner() {

    }

    void init();

    void setup();

    void loop();

    void stop();

    void startBluetoothScan(ScanType scanType);

    void addKnownDevice   (const std::string&  input);
    void addKnownDevice   (const esp_bd_addr_t mac, const char* alias);
    void deleteKnownDevice(const std::string&  mac);
    void deleteKnownDevice(const esp_bd_addr_t mac);

    // BLE advertisement results:
    void onResult(BLEAdvertisedDevice advertisedDevice);
    
    void HandleBleAdvertisementResult(BLEAdvertisedDevice& bleAdvertisedDeviceResult);

private:
    void loadSettings();
    void removeFromBtDevices(const esp_bd_addr_t mac);

    uint8_t getNumScans(ScanType scanType);
    unsigned long getLastScanTime(ScanType scanType);
    void setLastScanTime(ScanType scanType, unsigned long time);

    bool scanForNextDevice();

    void SetReadRemoteNameResult(const esp_bt_gap_cb_param_t::read_rmt_name_param& remoteNameParam);
    void HandleReadRemoteNameResult(esp_bt_gap_cb_param_t::read_rmt_name_param& remoteNameParam);

    void gap_init();
    void startupGap();
    void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    void update_device_info(esp_bt_gap_cb_param_t *param);

    struct btDeviceId_t {
        esp_bd_addr_t mac;
        String name;
        uint8_t confidence;
        uint8_t state;
        uint8_t scansLeft;

        btDeviceId_t(const esp_bd_addr_t MAC, String Name) : name(std::move(Name)), confidence(0), state(0)
        {
            memcpy(mac, MAC, sizeof(esp_bd_addr_t));
        }
    };
public:
    const std::vector<btDeviceId_t>& getBtDeviceStates();

private:

    typedef enum {
        APP_GAP_STATE_IDLE = 0,
        APP_GAP_STATE_DEVICE_DISCOVERING,
        APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
        APP_GAP_STATE_SERVICE_DISCOVERING,
        APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
    } app_gap_state_t;

    typedef struct {
        bool dev_found;
        uint8_t bdname_len;
        uint8_t eir_len;
        uint8_t rssi;
        uint32_t cod;
        uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
        uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
        esp_bd_addr_t bda;
        app_gap_state_t state;
    } app_gap_cb_t;


    app_gap_cb_t m_dev_info;
    std::vector<btDeviceId_t> btDevices;
    std::mutex btDevicesMutex;

    volatile uint8_t scanInProgress = 0;
    int scanIndex = -1; // loops over devices
    int iterationScansLeft = -1; // Loops over iterations
    ScanType scanMode;

    
    int scanTime = 1; //In seconds
    BLEScan *pBLEScan = nullptr;


    std::queue<esp_bt_gap_cb_param_t::read_rmt_name_param> readRemoteNameResultQueue;

    std::queue<BLEAdvertisedDevice> bleAdvertisedDeviceResultQueue;

    // Scanner parameters from storage:
    const char* scanner_identity;
    uint8_t num_arrival_scans;
    uint8_t num_departure_scans;
    unsigned long scan_iter_interval;
    uint32_t beacon_expiration_seconds;
    uint32_t min_seconds_between_scans;
    uint32_t periodic_scan_interval;

    unsigned long last_arrival_scan_time = 0;
    unsigned long last_departure_scan_time = 0;

    unsigned long last_scan_iter_millis = 0;
};

#endif // BLUETOOTH_SCANNER_H
