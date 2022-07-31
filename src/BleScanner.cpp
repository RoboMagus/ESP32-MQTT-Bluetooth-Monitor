// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <array>
#include <vector>
#include <utility>
#include <algorithm>

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

#ifndef ESPHOME
#include <Arduino.h>
#include <ezTime.h>

extern Timezone mTime;
#endif

// Tweaked SDK configuration
#include "sdkconfig.h"

#include "BleScanner.h"
#include "parameter.h"
#include "mqtt.h"
#include "led.h"

#include "stackDbgHelper.h"
#include "CallbackHelper.h"


#define GAP_TAG          "GAP"
#define SPP_TAG          "SPP"
#define GATTC_TAG        "GATTC"

#define BTSCAN_TAG       "BTSCAN"

#define BLE_SCAN_COMPLETE_CALLBACK_SIGNATURE void(BLEScanResults)

typedef void (*scanComplete_cb_t)(BLEScanResults);

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

static const unsigned long timeout = 1000*60; // 60sec

// =================================================================================
// =================================================================================
//      Free Functions:
// =================================================================================

// -----------------------------------------------
#define BLUETOOTH_MON_VERSION "0.1.001"
extern std::string createConfidenceMessage(const char* MAC, uint8_t confidence, const char* name, const char* timestamp, bool retain = false, const char* iBeaconStr = nullptr, const char* type = "KNOWN_MAC", const char* manufacturer = "Unknown");
// {
//     char buf[420];
//     // Wed Jun 09 2021 20:03:45 GMT+0200 (CEST)
//     snprintf(buf, 420, "{\"id\":\"%s\",%s%s\"confidence\":\"%d\",\"name\":\"%s\",\"manufacturer\":\"%s\",\"type\":\"%s\",\"retained\":\"%s\",\"timestamp\":\"%s\",\"version\":\"%s\"}", 
//         MAC, (iBeaconStr ? iBeaconStr : " "), (iBeaconStr ? "," : " "), confidence, name, manufacturer, type, (retain ? "true" : "false"), timestamp, BLUETOOTH_MON_VERSION);
//     return buf;
// }

// -----------------------------------------------
#ifdef ESPHOME
std::string getDateTimeString() {
    static const std::string nyi("Not yet implemented...");
    return nyi;
}
#else
String getDateTimeString() {
    return mTime.dateTime("D M d Y H:i:s ~G~M~TO (T)");
}
#endif

// -----------------------------------------------

void BleScanner::onResult(BLEAdvertisedDevice advertisedDevice) {
    bleAdvertisedDeviceResultQueue.push(advertisedDevice);
}

// -----------------------------------------------
void BleScanner::HandleBleAdvertisementResult(BLEAdvertisedDevice& bleAdvertisedDeviceResult) {
    SCOPED_STACK_ENTRY;
    if (bleAdvertisedDeviceResult.haveName()) {
       ESP_LOGI(BTSCAN_TAG, "Device name: %s", bleAdvertisedDeviceResult.getName().c_str());
    }

    if (bleAdvertisedDeviceResult.haveServiceUUID()) {
       // ESP_LOGI(BTSCAN_TAG, "Found ServiceUUID: %s", devUUID.toString().c_str());
    }
    else
    {
        if (bleAdvertisedDeviceResult.haveManufacturerData() == true)
        {
            std::string strManufacturerData = bleAdvertisedDeviceResult.getManufacturerData();

            uint8_t cManufacturerData[100];
            strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);

            if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C && cManufacturerData[1] == 0x00)
            {
                ESP_LOGD(BTSCAN_TAG, "Found an iBeacon!");
                std::string rssi_s = "??";
                if(bleAdvertisedDeviceResult.haveRSSI()) {
                    rssi_s = bleAdvertisedDeviceResult.getRSSI();
                }
                BLEBeacon oBeacon = BLEBeacon();
                oBeacon.setData(strManufacturerData);
                ESP_LOGD(BTSCAN_TAG, "iBeacon Frame");

                BLEUUID uuid_swapped(oBeacon.getProximityUUID().getNative()->uuid.uuid128, 16, true);
                ESP_LOGD(BTSCAN_TAG, "ID: %04X Major: %d Minor: %d UUID: %s Power: %d, RSSI: %s\n", oBeacon.getManufacturerId(), ENDIAN_CHANGE_U16(oBeacon.getMajor()), ENDIAN_CHANGE_U16(oBeacon.getMinor()), uuid_swapped.toString().c_str(), oBeacon.getSignalPower(), rssi_s.c_str());
                
                std::lock_guard<std::mutex> lock(iBeaconDevicesMutex);
                for (auto &iBeacon : iBeaconDevices){
                    if(iBeacon.uuid.equals(uuid_swapped)) {                        
                        if(bleAdvertisedDeviceResult.haveRSSI()) {
                            iBeacon.addRSSI(bleAdvertisedDeviceResult.getRSSI());
                        }
                        iBeacon.power = oBeacon.getSignalPower();
                        iBeacon.confidence = 100;
                        iBeacon.last_update_millis = millis();

                        int filteredRssi = iBeacon.getFilteredRSSI();
                        if(abs(filteredRssi - iBeacon.lastSentRssi) > 3) {
                            char rssi_str[25];
                            snprintf(rssi_str, 24, "\"rssi\":\"%d\"", filteredRssi);
                            iBeacon.lastSentRssi = filteredRssi;
                            std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + iBeacon.name.c_str();
                            mqtt.send_message(topic.c_str(), 
                                    createConfidenceMessage(iBeacon.uuid.toString().c_str(), iBeacon.confidence, iBeacon.name.c_str(), getDateTimeString().c_str(), m_retain, rssi_str, "KNOWN_BEACON" ).c_str(), m_retain
                                );
                        }
                        else {
                            ESP_LOGI(BTSCAN_TAG, "%s rssi: %d, last sent rssi: %d\n", iBeacon.name.c_str(), filteredRssi, iBeacon.lastSentRssi);
                        }
                    }
                }
            }
            else
            { /*
                mSerial.println("Found another manufacturers beacon!");
                mSerial.printf("strManufacturerData: %d ", strManufacturerData.length());
                for (int i = 0; i < strManufacturerData.length(); i++)
                {
                    mSerial.printf("[%X]", cManufacturerData[i]);
                }
                mSerial.printf("\n");
            */
            }
        }
        return;
    }

    uint8_t *payLoad = bleAdvertisedDeviceResult.getPayload();

    BLEUUID checkUrlUUID = (uint16_t)0xfeaa;

    if (bleAdvertisedDeviceResult.getServiceUUID().equals(checkUrlUUID))
    {
        if (payLoad[11] == 0x10)
        {
            ESP_LOGI(BTSCAN_TAG, "Found an EddystoneURL beacon!");
            BLEEddystoneURL foundEddyURL = BLEEddystoneURL();
            std::string eddyContent((char *)&payLoad[11]); // incomplete EddystoneURL struct!

            foundEddyURL.setData(eddyContent);
            std::string bareURL = foundEddyURL.getURL();
            if (bareURL[0] == 0x00)
            {
                size_t payLoadLen = bleAdvertisedDeviceResult.getPayloadLength();
                // mSerial.println("DATA-->");
                // for (int idx = 0; idx < payLoadLen; idx++)
                // {
                //     mSerial.printf("0x%08X ", payLoad[idx]);
                // }
                ESP_LOGI(BTSCAN_TAG, "\nInvalid Data");
                return;
            }

            ESP_LOGI(BTSCAN_TAG, "Found URL: %s\n", foundEddyURL.getURL().c_str());
            ESP_LOGI(BTSCAN_TAG, "Decoded URL: %s\n", foundEddyURL.getDecodedURL().c_str());
            ESP_LOGI(BTSCAN_TAG, "TX power %d\n", foundEddyURL.getPower());
        }
        else if (payLoad[11] == 0x20)
        {
            ESP_LOGI(BTSCAN_TAG, "Found an EddystoneTLM beacon!");
            BLEEddystoneTLM foundEddyURL = BLEEddystoneTLM();
            std::string eddyContent((char *)&payLoad[11]); // incomplete EddystoneURL struct!

            eddyContent = "01234567890123";

            for (int idx = 0; idx < 14; idx++)
            {
                eddyContent[idx] = payLoad[idx + 11];
            }

            foundEddyURL.setData(eddyContent);
            ESP_LOGI(BTSCAN_TAG, "Reported battery voltage: %dmV\n", foundEddyURL.getVolt());
            ESP_LOGI(BTSCAN_TAG, "Reported temperature from TLM class: %.2fC\n", (double)foundEddyURL.getTemp());
            int temp = (int)payLoad[16] + (int)(payLoad[15] << 8);
            float calcTemp = temp / 256.0f;
            ESP_LOGI(BTSCAN_TAG, "Reported temperature from data: %.2fC\n", calcTemp);
            ESP_LOGI(BTSCAN_TAG, "Reported advertise count: %d\n", foundEddyURL.getCount());
            ESP_LOGI(BTSCAN_TAG, "Reported time since last reboot: %ds\n", foundEddyURL.getTime());
            ESP_LOGI(BTSCAN_TAG, "%s", foundEddyURL.toString().c_str());
        }
    }
}


// =================================================================================
// =================================================================================
//      Class Functions:
// =================================================================================

// -----------------------------------------------
void BleScanner::init()
{
    // Setup static callback wrapper helper function for ble Scan Completed callback function:    
    Callback<BLE_SCAN_COMPLETE_CALLBACK_SIGNATURE>::func = std::bind(&BleScanner::bleScanCompleted, this, std::placeholders::_1);

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(this);
    pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
    pBLEScan->setInterval(200);
    pBLEScan->setWindow(160); // less or equal setInterval value
    // Scan for 100ms, then not for 25ms
    
    bleScan_shouldStart = true;
}

// -----------------------------------------------
void BleScanner::setup()
{
    // BT init malloc failure: 
}

// -----------------------------------------------
void BleScanner::bleScanCompleted(BLEScanResults)
{
    ESP_LOGI(BTSCAN_TAG, ">> bleScanCompleted()");
    
    bleScan_shouldStart = true;
}

// -----------------------------------------------
void BleScanner::loop()
{
    SCOPED_STACK_ENTRY;

    if(!bleAdvertisedDeviceResultQueue.empty()) {
        // mSerial.printf("Found %d ble devices\n", bleAdvertisedDeviceResultQueue.size());
        for(uint8_t i = 0; i < min(maxBleProcessPerIteration, bleAdvertisedDeviceResultQueue.size()); i++){
            HandleBleAdvertisementResult(bleAdvertisedDeviceResultQueue.front());
            bleAdvertisedDeviceResultQueue.pop();
            vTaskDelay(1);
        }
    }

    // Start new ble scan only if not performing regular scan
    if(bleScan_shouldStart){
        pBLEScan->stop();
        //pBLEScan->clearResults(); Taken care of when starting with continue!
        
        scanContinueCount = (scanContinueCount+1)%scanContinueWraparound;
        if(scanContinueCount == 0) {
            ESP_LOGI(BTSCAN_TAG, "Starting clean BLE scan");
        }
        if(!pBLEScan->start(scanTime, static_cast<scanComplete_cb_t>(Callback<BLE_SCAN_COMPLETE_CALLBACK_SIGNATURE>::callback), scanContinueCount)) {
            ESP_LOGE(BTSCAN_TAG, "Problem starting BLE scan!!");
        }
        else{
            bleScan_shouldStart = false;
        }
    }

    // Check for outdated ble RSSI measurements and send update if needed:
    const auto _millis = millis();
    const unsigned long iBeaconTimeout = 30000; // 30s
    for (auto &iBeacon : iBeaconDevices){
        if(iBeacon.last_update_millis + iBeaconTimeout < _millis && !iBeacon.isVirgin()) {
            iBeacon.reset();
            ESP_LOGI(BTSCAN_TAG, "Lost beacon '%s'. Resetting.\n", iBeacon.name.c_str());
            
            int filteredRssi = iBeacon.getFilteredRSSI();
            if(abs(filteredRssi - iBeacon.lastSentRssi) > 3) {
                char rssi_str[25] = "\"rssi\":\"\"";
                // snprintf(rssi_str, 24, "\"rssi\":\"\"", filteredRssi);
                iBeacon.lastSentRssi = filteredRssi;
                std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + iBeacon.name.c_str();
                mqtt.send_message(topic.c_str(), 
                        createConfidenceMessage(iBeacon.uuid.toString().c_str(), iBeacon.confidence, iBeacon.name.c_str(), getDateTimeString().c_str(), m_retain, rssi_str, "KNOWN_BEACON").c_str(), m_retain
                    );
            }
        }
    }
}

// -----------------------------------------------
void BleScanner::stop() {
    
}

// -----------------------------------------------
void BleScanner::setBeaconExpiration(uint32_t val) {
    beacon_expiration_seconds = val;

}

// -----------------------------------------------
void BleScanner::setMinTimeBetweenScans(uint32_t val) {
    min_seconds_between_scans = val;
}

// -----------------------------------------------
void BleScanner::setPeriodicScanInterval(uint32_t val) {
    periodic_scan_interval = val;
}

// -----------------------------------------------
void BleScanner::setMqttTopic(const std::string& topic){
    m_mqtt_topic = topic;
}

// -----------------------------------------------
void BleScanner::setScannerIdentity(const char* identity){
    m_scanner_identity = identity;
}

// -----------------------------------------------
void BleScanner::setRetainFlag(bool flag) {
    m_retain = flag;
}

// -----------------------------------------------
void BleScanner::startBluetoothScan() 
{
    SCOPED_STACK_ENTRY;
}

// -----------------------------------------------
void BleScanner::addKnownDevice(const std::string& input) {
    std::string _in = input;
    _in.erase(_in.find_last_not_of(" \t\n\v") + 1);
    std::string identifier = _in.substr(0, _in.find(" "));
    std::string alias = _in.substr(_in.find(" "));

    size_t first = alias.find_first_not_of(" \t\n\v");
    size_t last = alias.find_last_not_of(" \t\n\v");
    if(first != std::string::npos) {
        alias = alias.substr(first, (last-first+1));
    }

    BLEUUID ble_uuid = BLEUUID::fromString(identifier);

    if (!ble_uuid.equals(BLEUUID())) {
        addKnownIBeacon(ble_uuid, alias.c_str());
    }
}

// -----------------------------------------------
void BleScanner::addKnownIBeacon(const BLEUUID uuid, const char* alias) {
    // Remove entry if it already existed in order to overwrite with updated one:
    deleteKnownIBeacon(uuid);

    {
        // Add to the set of currently known iBeacons:
        std::lock_guard<std::mutex> lock(iBeaconDevicesMutex);

        iBeaconDevices.emplace_back(uuid, alias);
        ESP_LOGI(BTSCAN_TAG, "Added iBeacon '%s'\n", alias);
    }
}

// -----------------------------------------------
void BleScanner::deleteKnownIBeacon(const BLEUUID uuid) {
    std::lock_guard<std::mutex> lock(iBeaconDevicesMutex);
    for (auto it = iBeaconDevices.begin(); it != iBeaconDevices.end(); /* NOTHING */) {
        if ((*it).uuid.equals(uuid)) {
            it = iBeaconDevices.erase(it);
        }
        else{
            ++it;
        }
    } 
}