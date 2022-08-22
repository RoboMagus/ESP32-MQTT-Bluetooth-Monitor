#ifndef BLUETOOTH_MONITOR_H
#define BLUETOOTH_MONITOR_H

#include <functional>

#include "BtClassicScanner.h"
#include "BleScanner.h"

// ------------------------------
// BtMonitor: A wrapper class that encompases both BLE and BT Classic scanner implementations
class BtMonitor
{
public:
    BtMonitor(Stream& serialStream) : m_bleScanner(serialStream) {
    }

    // Base function wrappers
    void init();
    void setup();
    void loop();
    void stop();
    
    // Device registration functions
    void addKnownDevice             (const std::string& input);
    void addKnownDevice             (const std::string& identifier, const std::string& alias);
    void deleteKnownDevice          (const std::string& identifier);
    void clearKnownDevices          ();

    // MQTT setup functions
    void setMqttTopic               (const std::string&   topic   );
    void setScannerIdentity         (const char*          identity);
    void setRetainFlag              (bool                 flag    );

    // BT Classic parameter setup functions    
    void setNumArrivalScans         (uint8_t              val     );
    void setNumDepartureScans       (uint8_t              val     );
    void setSecondsBetweenScanIters (unsigned long        val     );
    void setMinTimeBetweenScans     (uint32_t             val     );
    void setPeriodicScanInterval    (uint32_t             val     );
    void setScanDurationTimeout     (uint32_t             val     );

    // BLE parameter setup functions
    void setBeaconExpiration        (uint32_t             val     );


    // Interactive functions
    void startBluetoothScan         (ScanType scanType);

    const std::vector<btDeviceId_t>& getBtDeviceStates() {
        return m_classicScanner.getBtDeviceStates();
    }

private:
    void onBtClassicDeviceScanStart(const btDeviceId_t&);
    void onBtClassicDeviceUpdate   (const btDeviceId_t&);

    void onBleDeviceUpdate         (const iBeaconDeviceId_t&);

private:
    BtClassicScanner m_classicScanner;
    BleScanner       m_bleScanner;

    // MQTT params
    std::string m_mqtt_topic;
    const char* m_scanner_identity;
    bool        m_retain = false;
};

#endif // BLUETOOTH_MONITOR_H