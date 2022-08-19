
#include <Arduino.h>
#include <ezTime.h>

#include "BtMonitor.h"
#include "mqtt.h"


extern Timezone mTime;

// =================================================================================
// =================================================================================
//      Free Functions:
// =================================================================================
namespace{
        
    // -----------------------------------------------
    #define BLUETOOTH_MON_VERSION "0.1.001"
    std::string createConfidenceMessage(const char* MAC, const uint8_t confidence, const char* name, const char* timestamp, bool retain = false, const char* iBeaconStr = nullptr, const char* type = "KNOWN_MAC", const char* manufacturer = "Unknown") {
        char buf[420];
        // Wed Jun 09 2021 20:03:45 GMT+0200 (CEST)
        snprintf(buf, 420, "{\"id\":\"%s\",%s%s\"confidence\":\"%d\",\"name\":\"%s\",\"manufacturer\":\"%s\",\"type\":\"%s\",\"retained\":\"%s\",\"timestamp\":\"%s\",\"version\":\"%s\"}", 
            MAC, (iBeaconStr ? iBeaconStr : " "), (iBeaconStr ? "," : " "), confidence, name, manufacturer, type, (retain ? "true" : "false"), timestamp, BLUETOOTH_MON_VERSION);
        return buf;
    }

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
    
} // namespace


// -----------------------------------------------
void BtMonitor::init(){
    m_classicScanner.setDeviceUpdateCallback(std::bind(&BtMonitor::onBtClassicDeviceUpdate, this, std::placeholders::_1));
    m_bleScanner.setDeviceUpdateCallback(std::bind(&BtMonitor::onBleDeviceUpdate, this, std::placeholders::_1));

    m_classicScanner.init();
    m_bleScanner.init();
}

// -----------------------------------------------
void BtMonitor::setup(){
    m_classicScanner.setup();
    m_bleScanner.setup();
}

// -----------------------------------------------
void BtMonitor::loop(){
    m_classicScanner.loop();
    m_bleScanner.loop();
}

// -----------------------------------------------
void BtMonitor::stop(){
    m_classicScanner.stop();
    m_bleScanner.stop();
}

// -----------------------------------------------
void BtMonitor::startBluetoothScan(ScanType scanType) {
    m_classicScanner.startBluetoothScan(scanType);
}

// -----------------------------------------------
void BtMonitor::onBtClassicDeviceUpdate(const btDeviceId_t& dev) {
    char macbuf[19];
    std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + dev.name.c_str();
    mqtt.send_message(topic.c_str(), 
            createConfidenceMessage(bda2str(dev.mac, macbuf, 18), dev.confidence, dev.name.c_str(), getDateTimeString().c_str(), m_retain).c_str(), m_retain
        );
}

// -----------------------------------------------
void BtMonitor::onBleDeviceUpdate(const iBeaconDeviceId_t& iBeacon) {    
    char rssi_str[25];
    snprintf(rssi_str, 24, "\"rssi\":\"%d\"", iBeacon.getFilteredRSSI());
    std::string topic = m_mqtt_topic + "/" + m_scanner_identity + "/" + iBeacon.name.c_str();
    BLEUUID uuid = iBeacon.uuid;
    mqtt.send_message(topic.c_str(), 
            createConfidenceMessage(uuid.toString().c_str(), iBeacon.confidence, iBeacon.name.c_str(), getDateTimeString().c_str(), m_retain, rssi_str, "KNOWN_BEACON" ).c_str(), m_retain
        );
}

// ===============================================
// ===============================================
// MQTT setup functions    
// -----------------------------------------------
void BtMonitor::setMqttTopic(const std::string& topic){
    m_mqtt_topic = topic;
}

// -----------------------------------------------
void BtMonitor::setScannerIdentity(const char* identity){
    m_scanner_identity = identity;
}

// -----------------------------------------------
void BtMonitor::setRetainFlag(bool flag) {
    m_retain = flag;
}


// ===============================================
// ===============================================
// BT Classic parameter setup functions    
// -----------------------------------------------
void BtMonitor::setNumArrivalScans (uint8_t val) {
    m_classicScanner.setNumArrivalScans(val);
}
// -----------------------------------------------
void BtMonitor::setNumDepartureScans (uint8_t val) {
    m_classicScanner.setNumDepartureScans(val);
}
// -----------------------------------------------
void BtMonitor::setSecondsBetweenScanIters (unsigned long val) {
    m_classicScanner.setSecondsBetweenScanIters(val);
}
// -----------------------------------------------
void BtMonitor::setMinTimeBetweenScans (uint32_t val) {
    m_classicScanner.setMinTimeBetweenScans(val);
}
// -----------------------------------------------
void BtMonitor::setPeriodicScanInterval (uint32_t val) {
    m_classicScanner.setPeriodicScanInterval(val);
}
// -----------------------------------------------
void BtMonitor::setScanDurationTimeout (uint32_t val) {
    m_classicScanner.setScanDurationTimeout(val);
}


// ===============================================
// ===============================================
// Device registration functions
// -----------------------------------------------
void BtMonitor::addKnownDevice(const std::string& input) {
    std::string _in = input;
    _in.erase(_in.find_last_not_of(" \t\n\v") + 1);
    std::string identifier = _in.substr(0, _in.find(" "));
    std::string alias = _in.substr(_in.find(" "));

    size_t first = alias.find_first_not_of(" \t\n\v");
    size_t last = alias.find_last_not_of(" \t\n\v");
    if(first != std::string::npos) {
        alias = alias.substr(first, (last-first+1));
    }

    addKnownDevice(identifier, alias);
}

// -----------------------------------------------
void BtMonitor::addKnownDevice(const std::string& identifier, const std::string& alias) {
    BLEUUID ble_uuid = BLEUUID::fromString(identifier);

    esp_bd_addr_t addr;
    if(str2bda(identifier.c_str(), addr)) {
        m_classicScanner.addKnownDevice(addr, alias.c_str());
    }
    else if (!ble_uuid.equals(BLEUUID())) {
        m_bleScanner.addKnownDevice(ble_uuid, alias.c_str());
    }
}

// -----------------------------------------------
void BtMonitor::deleteKnownDevice(const std::string& identifier) {
    BLEUUID ble_uuid = BLEUUID::fromString(identifier);
    esp_bd_addr_t addr;
    if(str2bda(identifier.c_str(), addr)) {
        m_classicScanner.deleteKnownDevice(addr);
    }
    else if (!ble_uuid.equals(BLEUUID())) {
        m_bleScanner.deleteKnownDevice(ble_uuid);
    }
}

// -----------------------------------------------
void BtMonitor::clearKnownDevices() {
    m_classicScanner.clearKnownDevices();
    m_bleScanner.clearKnownDevices();
}
