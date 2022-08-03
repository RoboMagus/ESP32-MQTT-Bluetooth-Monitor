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

#include <Arduino.h>
#include <ezTime.h>

// Tweaked SDK configuration
#include "sdkconfig.h"

#include "BluetoothScanner.h"
#include "parameter.h"
#include "mqtt.h"
#include "led.h"

#include "stackDbgHelper.h"
#include "CallbackHelper.h"


#define GAP_TAG          "GAP"
#define SPP_TAG          "SPP"
#define GATTC_TAG        "GATTC"

#define BTSCAN_TAG       "BTSCAN"

#define BLE_SCAN_AFTER_READ_REMOTE_NAMES (0)


#define BLE_SCAN_COMPLETE_CALLBACK_SIGNATURE void(BLEScanResults)
#define GAP_CALLBACK_SIGNATURE               void(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)

typedef void (*scanComplete_cb_t)(BLEScanResults);


extern Timezone mTime;

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

// =================================================================================
// =================================================================================
//      Free Functions:
// =================================================================================

// -----------------------------------------------
#define BLUETOOTH_MON_VERSION "0.1.001"
String createConfidenceMessage(const char* MAC, uint8_t confidence, const char* name, const char* timestamp, bool retain = false, const char* iBeaconStr = nullptr, const char* type = "KNOWN_MAC", const char* manufacturer = "Unknown") {
    char buf[420];
    // Wed Jun 09 2021 20:03:45 GMT+0200 (CEST)
    snprintf(buf, 420, "{\"id\":\"%s\",%s%s\"confidence\":\"%d\",\"name\":\"%s\",\"manufacturer\":\"%s\",\"type\":\"%s\",\"retained\":\"%s\",\"timestamp\":\"%s\",\"version\":\"%s\"}", 
        MAC, (iBeaconStr ? iBeaconStr : " "), (iBeaconStr ? "," : " "), confidence, name, manufacturer, type, (retain ? "true" : "false"), timestamp, BLUETOOTH_MON_VERSION);
    return buf;
}

// -----------------------------------------------
char *bda2str(const esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    const uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

// -----------------------------------------------
bool str2bda(const char* str, esp_bd_addr_t& mac) {
    if(strlen(str) < 12 || strlen(str) > 18) {
        return false;
    }

    uint8_t *p = mac;
    // Scan for MAC with semicolon separators
    int args_found = sscanf(str, "%" SCNx8 ":%" SCNx8 ":%" SCNx8 ":%" SCNx8 ":%" SCNx8 ":%" SCNx8,
            &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);
    if( args_found == 6 ) {
        return true;
    }

    // Scan for mac without semicolons
    args_found = sscanf(str, "%" SCNx8 "%" SCNx8 "%" SCNx8 "%" SCNx8 "%" SCNx8 "%" SCNx8,
            &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);
    
    if( args_found == 6 ) {
        return true;
    }

    // Invalid MAC 
    return false;
}

// -----------------------------------------------
static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
{
    if (uuid == NULL || str == NULL) {
        return NULL;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08x", uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
                p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return NULL;
    }

    return str;
}

// -----------------------------------------------
static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

// -----------------------------------------------

void BluetoothScanner::onResult(BLEAdvertisedDevice advertisedDevice) {
    bleAdvertisedDeviceResultQueue.push(advertisedDevice);
}

// -----------------------------------------------
void BluetoothScanner::HandleBleAdvertisementResult(BLEAdvertisedDevice& bleAdvertisedDeviceResult) {
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
                String rssi_s = "??";
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
                                    createConfidenceMessage(iBeacon.uuid.toString().c_str(), iBeacon.confidence, iBeacon.name.c_str(), mTime.dateTime("D M d Y H:i:s ~G~M~TO (T)").c_str(), m_retain, rssi_str, "KNOWN_BEACON" ).c_str(), m_retain
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
void BluetoothScanner::update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(GAP_TAG, "Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    app_gap_cb_t *p_dev = &m_dev_info;
    if (p_dev->dev_found && 0 != memcmp(param->disc_res.bda, p_dev->bda, ESP_BD_ADDR_LEN)) {
        return;
    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE)) {
        return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            p_dev->cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            p_dev->rssi = *(int8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            uint8_t len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            memcpy(p_dev->bdname, (uint8_t *)(p->val), len);
            p_dev->bdname[len] = '\0';
            p_dev->bdname_len = len;
            break;
        }
        case ESP_BT_GAP_DEV_PROP_EIR: {
            memcpy(p_dev->eir, (uint8_t *)(p->val), p->len);
            p_dev->eir_len = p->len;
            break;
        }
        default:
            break;
        }
    }

    if (p_dev->eir && p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
        ESP_LOGI(GAP_TAG, "Found a target device, address %s, name %s", bda_str, p_dev->bdname);
        p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
        ESP_LOGI(GAP_TAG, "Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}

// -----------------------------------------------
void BluetoothScanner::gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    char bda_str[18];
    char uuid_str[37];

    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        update_device_info(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(GAP_TAG, "Device discovery stopped.");
            if ( (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE ||
                    p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
                    && p_dev->dev_found) {
                p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
                ESP_LOGI(GAP_TAG, "Discover services ...");
                esp_bt_gap_get_remote_services(p_dev->bda);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(GAP_TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
                p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
                for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
                    esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
                    ESP_LOGI(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
                    // ESP_LOGI(GAP_TAG, "--%d", u->len);
                }
            } else {
                ESP_LOGI(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
            }
        }
        break;
    }
    case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
        ESP_LOGI(GAP_TAG, "Remote device status: %d, addr: %s, rssi: %d", param->read_rssi_delta.stat,  bda2str(param->read_rssi_delta.bda, bda_str, 18),  param->read_rssi_delta.rssi_delta);
    break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        SetReadRemoteNameResult(param->read_rmt_name);
    break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default: {
        ESP_LOGI(GAP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

// -----------------------------------------------
void BluetoothScanner::init()
{
    esp_err_t ret;

// No mem release allows BT and BLE to work side-by-side, but is there enough mem left for the rest...?
    //esp_bt_mem_release(ESP_BT_MODE_BTDM);
    // esp_bt_controller_mem_release(ESP_BT_MODE_BLE);  // --> This fixes MEM Alloc failures in esp_bluedroid_enable!!!
#if 1
    // Call bluetooth start from esp32 arduino framework 
    if(!btStarted() && !btStart()){
        log_e("btStart failed");
        return;
    }

    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
    if(bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        if ((ret = esp_bluedroid_init()) != ESP_OK) {
            ESP_LOGE(GAP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
    }

    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            ESP_LOGE(GAP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
    }
            
    gap_init();
    startupGap();

#endif

    // Setup static callback wrapper helper function for ble Scan Completed callback function:    
    Callback<BLE_SCAN_COMPLETE_CALLBACK_SIGNATURE>::func = std::bind(&BluetoothScanner::bleScanCompleted, this, std::placeholders::_1);

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
void BluetoothScanner::setup()
{
    // BT init malloc failure: 

// config.c L537
// static void config_parse(nvs_handle_t fp, config_t *config)
// ...
//  int buf_size = get_config_size_from_flash(fp);
//  char *buf = NULL;
//  if(buf_size == 0) { //First use nvs
//      goto error;
//  }
//  buf = osi_calloc(buf_size);

}

// -----------------------------------------------
void BluetoothScanner::bleScanCompleted(BLEScanResults)
{
    ESP_LOGI(BTSCAN_TAG, ">> bleScanCompleted()");
    
    bleScan_shouldStart = true;
}

// -----------------------------------------------
void BluetoothScanner::loop()
{
    SCOPED_STACK_ENTRY;
    // Handle read remote name result outside of callback if made available:
    if(!readRemoteNameResultQueue.empty()) {
        HandleReadRemoteNameResult(readRemoteNameResultQueue.front());
        readRemoteNameResultQueue.pop();
    }

    if(!bleAdvertisedDeviceResultQueue.empty()) {
        // mSerial.printf("Found %d ble devices\n", bleAdvertisedDeviceResultQueue.size());
        for(uint8_t i = 0; i < min(maxBleProcessPerIteration, bleAdvertisedDeviceResultQueue.size()); i++){
            HandleBleAdvertisementResult(bleAdvertisedDeviceResultQueue.front());
            bleAdvertisedDeviceResultQueue.pop();
            vTaskDelay(1);
        }
    }

    unsigned long current_millis = millis();

    // If scan is still active but timed out...
    if(scanMode != ScanType::None) {
        if(current_millis - getLastScanTime(scanMode) > scan_duration_timeout) {
            led.set(OFF);
            ESP_LOGW(BTSCAN_TAG, "Scan timed out. Reseting scanning state!\n");
            scanIndex = -1;
            scanMode = ScanType::None;
        }
    }

    if(scanMode != ScanType::None && !scanInProgress && btDevices.size() != 0) {
        if(current_millis > last_scan_iter_millis + scan_iter_interval || scanIndex != 0) { // only block first device of the list!
            last_scan_iter_millis = current_millis;

            // ToDo: walk scanning state machine if scan active...

            // Loop over devices to be scanned:
            uint8_t devices_checked;
            for (devices_checked = 0; devices_checked <= btDevices.size(); devices_checked++) {
                scanIndex = (scanIndex+1)%btDevices.size();
                if(scanIndex == 0) {
                    // If we loop over the devices to scan, we start a new iteration!
                    iterationScansLeft--;
                }

                if(scanForNextDevice()) {
                    break;
                }
            }
            if (devices_checked >= btDevices.size()) { // All devices scanned. No more scanning
                ESP_LOGI(BTSCAN_TAG, "Stopping All searches...\n\n");
                scanIndex = -1;
                scanMode = ScanType::None;

                bleScan_shouldStart = true; 
            }
        }   
    }   

    // Check for periodic scan:
    if(periodic_scan_interval > 0) {
        if(current_millis > getLastScanTime(ScanType::Either) + periodic_scan_interval*1000) {
            startBluetoothScan(ScanType::Either);
        }
    }

    // Start new ble scan only if not performing regular scan
    if(bleScan_shouldStart && scanMode == ScanType::None){
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
                        createConfidenceMessage(iBeacon.uuid.toString().c_str(), iBeacon.confidence, iBeacon.name.c_str(), mTime.dateTime("D M d Y H:i:s ~G~M~TO (T)").c_str(), m_retain, rssi_str, "KNOWN_BEACON").c_str(), m_retain
                    );
            }
        }
    }
}

// -----------------------------------------------
void BluetoothScanner::stop() {
    btStop();
}

// -----------------------------------------------
uint8_t BluetoothScanner::getNumScans(ScanType scanType) {
    uint8_t numScans = 0;
    switch(scanType) {
        case ScanType::Either:
            numScans = max(num_arrival_scans, num_departure_scans);
        break;

        case ScanType::Arrival:
            numScans = num_arrival_scans;
        break;

        case ScanType::Departure:
            numScans = num_departure_scans;
        break;

        case ScanType::None:
        default:
            numScans = 0;
    }
    return numScans;
}

// -----------------------------------------------
unsigned long BluetoothScanner::getLastScanTime(ScanType scanType) {
    switch(scanType) {
        case ScanType::Arrival:
            return last_arrival_scan_time;
            break;
        case ScanType::Departure:
            return last_departure_scan_time;
            break;
        case ScanType::Either:
            return min(last_departure_scan_time, last_arrival_scan_time);
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

// -----------------------------------------------
void BluetoothScanner::setLastScanTime(ScanType scanType, unsigned long time) {
    switch(scanType) {
        case ScanType::Arrival:
            last_arrival_scan_time = time;
            break;
        case ScanType::Departure:
            last_departure_scan_time = time;
            break;
        case ScanType::Either:
            last_departure_scan_time = time;
            last_arrival_scan_time = time;
            break;
        default:
            break;
    }
}

// -----------------------------------------------
void BluetoothScanner::setNumArrivalScans(uint8_t val) {
    num_arrival_scans = val;
}

// -----------------------------------------------
void BluetoothScanner::setNumDepartureScans(uint8_t val) {
    num_departure_scans = val;
}

// -----------------------------------------------
void BluetoothScanner::setSecondsBetweenScanIters(unsigned long val) {
    scan_iter_interval = val;
}

// -----------------------------------------------
void BluetoothScanner::setBeaconExpiration(uint32_t val) {
    beacon_expiration_seconds = val;

}

// -----------------------------------------------
void BluetoothScanner::setMinTimeBetweenScans(uint32_t val) {
    min_seconds_between_scans = val;
}

// -----------------------------------------------
void BluetoothScanner::setPeriodicScanInterval(uint32_t val) {
    periodic_scan_interval = val;
}

// -----------------------------------------------
void BluetoothScanner::setScanDurationTimeout(uint32_t val) {
    scan_duration_timeout = val*1000;
}

// -----------------------------------------------
void BluetoothScanner::setMqttTopic(const std::string& topic){
    m_mqtt_topic = topic;
}

// -----------------------------------------------
void BluetoothScanner::setScannerIdentity(const char* identity){
    m_scanner_identity = identity;
}

// -----------------------------------------------
void BluetoothScanner::setRetainFlag(bool flag) {
    m_retain = flag;
}

// -----------------------------------------------
void BluetoothScanner::startBluetoothScan(ScanType scanType) 
{
    SCOPED_STACK_ENTRY;
    uint8_t numScans = getNumScans(scanType);
    unsigned long reference_scan_time = getLastScanTime(scanType);
    unsigned long _millis = millis();

    // Only scan if the last scan was long ago enough...  
    if( _millis > reference_scan_time + min_seconds_between_scans*1000 && scanMode == ScanType::None) {        
        ESP_LOGI(BTSCAN_TAG, "Starting search for devices with scantype %d\n  Set numScans to %d\n", (int)(scanType), numScans);

        scanMode = scanType;

        // Stop ble scan when starting regular scan:
        // pBLEScan->stop();

        std::lock_guard<std::mutex> lock(btDevicesMutex);
        // Set counter for all devices:
        iterationScansLeft = numScans;
        for( auto& dev : btDevices ) {
            if(scanMode == ScanType::Arrival && dev.state == 1) { // Device already present so only scan for departure
                dev.scansLeft = 0;
            }
            else if (scanMode == ScanType::Departure && dev.state == 0) { // Device not present, only scan for arrival
                dev.scansLeft = 0;
            }
            else {
                // ScanMode Either will always scan regardless of state.
                dev.scansLeft = numScans;
            }
        }

        // Start scanning at the beginning:
        scanIndex = 0;
        scanForNextDevice();

        setLastScanTime(scanMode, _millis);
        last_scan_iter_millis = _millis;
    }
    else {
        ESP_LOGI(BTSCAN_TAG, "Skipping BT scan. Too soon dude...");
        ESP_LOGI(BTSCAN_TAG, "milllis %lu < %lu, scanIdx: %d\n", _millis, reference_scan_time + min_seconds_between_scans*1000, scanIndex);
    }
}

// -----------------------------------------------
bool BluetoothScanner::scanForNextDevice() {
    if(btDevices.size()) {
        auto& dev = btDevices.at(scanIndex);
        if(dev.scansLeft > 0) {
            led.set(128);
            scanInProgress = 1;
            esp_bt_gap_read_remote_name(dev.mac);
            ESP_LOGI(BTSCAN_TAG, "Searching device: %s\n", dev.name.c_str());
            return true;
        }
    }
    return false;
}

// -----------------------------------------------
void BluetoothScanner::addKnownDevice(const std::string& input) {
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

    esp_bd_addr_t addr;
    if(str2bda(identifier.c_str(), addr)) {
        addKnownDevice(addr, alias.c_str());
    }
    else if (!ble_uuid.equals(BLEUUID())) {
        addKnownIBeacon(ble_uuid, alias.c_str());
    }
}

// -----------------------------------------------
void BluetoothScanner::addKnownIBeacon(const BLEUUID uuid, const char* alias) {
    // Remove entry if it already existed in order to overwrite with updated one:
    deleteKnownIBeacon(uuid);

    {
        // Add to the set of currently known iBeacons:
        std::lock_guard<std::mutex> lock(iBeaconDevicesMutex);

        iBeaconDevices.emplace_back(uuid, String(alias));
        ESP_LOGI(BTSCAN_TAG, "Added iBeacon '%s'\n", alias);
    }
}

// -----------------------------------------------
void BluetoothScanner::deleteKnownIBeacon(const BLEUUID uuid) {
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

// -----------------------------------------------
void BluetoothScanner::addKnownDevice(const esp_bd_addr_t mac, const char* alias) {
    char bda_str[18];
    if (bda2str(mac, bda_str, 18)) {
        // Add to the set of currently known devices:
        std::lock_guard<std::mutex> lock(btDevicesMutex);
        // Remove entry if it already existed in order to overwrite with updated one:
        removeFromBtDevices(mac);
        btDevices.emplace_back(mac, String(alias));
    }
}

// -----------------------------------------------
void BluetoothScanner::deleteKnownDevice(const std::string& mac) {
    esp_bd_addr_t addr;
    if(str2bda(mac.c_str(), addr)) {
        deleteKnownDevice(addr);
    }
}

// -----------------------------------------------
void BluetoothScanner::removeFromBtDevices(const esp_bd_addr_t mac) {
    for (auto it = btDevices.begin(); it != btDevices.end(); /* NOTHING */) {
        if (memcmp((*it).mac, mac, sizeof(esp_bd_addr_t)) == 0) {
            it = btDevices.erase(it);
        }
        else{
            ++it;
        }
    }  
}

// -----------------------------------------------
void BluetoothScanner::deleteKnownDevice(const esp_bd_addr_t mac) {
    std::lock_guard<std::mutex> lock(btDevicesMutex);
    removeFromBtDevices(mac);
}


// -----------------------------------------------
void BluetoothScanner::clearKnownDevices() {
    std::lock_guard<std::mutex> lock(btDevicesMutex);
    btDevices.clear();
}


// -----------------------------------------------
const std::vector<BluetoothScanner::btDeviceId_t>& BluetoothScanner::getBtDeviceStates() {
    std::lock_guard<std::mutex> lock(btDevicesMutex);
    return btDevices;
}

// -----------------------------------------------
void BluetoothScanner::SetReadRemoteNameResult(const esp_bt_gap_cb_param_t::read_rmt_name_param& remoteNameParam) {
    readRemoteNameResultQueue.push(remoteNameParam);
}

// -----------------------------------------------
void BluetoothScanner::HandleReadRemoteNameResult(esp_bt_gap_cb_param_t::read_rmt_name_param& remoteNameParam) {
    SCOPED_STACK_ENTRY;
    char macbuf[19];
    led.set(OFF);
    // If more scans are to be done, this causes the main loop to pick it up again!
    scanInProgress = 0; 

    std::lock_guard<std::mutex> lock(btDevicesMutex);
    btDeviceId_t& dev = btDevices.at(scanIndex);

    ESP_LOGI(BTSCAN_TAG, "ScanResult Callback for dev(%d): %s, scansLeft %d \n", scanIndex, dev.name.c_str(), dev.scansLeft);

    // Decrement scansLeft for each finished scan:
    if (dev.scansLeft > 0) {
        dev.scansLeft--;
        ESP_LOGI(BTSCAN_TAG, "Decrement scansleft for dev(%d): %s to %d \n", scanIndex, dev.name.c_str(), dev.scansLeft);
    }

    // Device found:
    if (ESP_BT_STATUS_SUCCESS == remoteNameParam.stat) {
        dev.confidence = 100;
        dev.state = 1;

        if(scanMode == ScanType::Arrival) {
            ESP_LOGI(BTSCAN_TAG, "Found device, stopping Arrival Scan for %s\n", dev.name.c_str());
            dev.scansLeft = 0; // Stop scanning, We've found him!
        }

        ESP_LOGI(BTSCAN_TAG, "Remote device name: %s", remoteNameParam.rmt_name);
        std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + dev.name.c_str();
        mqtt.send_message(topic.c_str(), 
                createConfidenceMessage(bda2str(dev.mac, macbuf, 18), dev.confidence, dev.name.c_str(), mTime.dateTime("D M d Y H:i:s ~G~M~TO (T)").c_str(), m_retain).c_str(), m_retain
            );
    }
    else {
        // Odd formula, but based on original in BluetoothPresence scripts...
        if(scanMode != ScanType::Arrival) {
            uint8_t confidence =  min(100.0f, (90.0f * dev.scansLeft) / float(num_departure_scans));
            dev.confidence = min(dev.confidence, confidence); // Dont increase confidence on departure scan.
        }
        else {
            dev.confidence = 0;
        }

        if(dev.scansLeft == 0) {
            dev.state = 0;
            dev.confidence = 0;
        }
        ESP_LOGI(GAP_TAG, "Remote device name read failed. Status: %d", remoteNameParam.stat);
        std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + dev.name.c_str();
        mqtt.send_message(topic.c_str(), 
                createConfidenceMessage(bda2str(dev.mac, macbuf, 18), dev.confidence, dev.name.c_str(), mTime.dateTime("D M d Y H:i:s ~G~M~TO (T)").c_str(), m_retain).c_str(), m_retain
            );
    }
    // Next scan is triggered from main loop...
} 

// -----------------------------------------------
void BluetoothScanner::gap_init()
{
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));
}

// -----------------------------------------------
void BluetoothScanner::startupGap(void)
{
    const char *dev_name = "ESP32_scanner";
    esp_bt_dev_set_device_name(dev_name);

    // set discoverable and connectable mode, wait to be connected 
    // esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    // register GAP callback function
    Callback<GAP_CALLBACK_SIGNATURE>::func = std::bind(&BluetoothScanner::gap_callback, this, std::placeholders::_1, std::placeholders::_2);
    esp_bt_gap_cb_t callback_func = static_cast<esp_bt_gap_cb_t>(Callback<GAP_CALLBACK_SIGNATURE>::callback);      
    esp_bt_gap_register_callback(callback_func);

    // inititialize device information and status
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    // start to discover nearby Bluetooth devices 
    // p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
    // esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 20, 0);
}
