#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"

#include "BtClassicScanner.h"

namespace esphome {
namespace bt_scanner {

class BluetoothClassicScanner : public Component {
 public:
  BluetoothClassicScanner() {}

  void setup() {
    _scanner.init();
    _scanner.setup();

    _scanner.setDeviceScanStartCallback(std::bind(&BluetoothClassicScanner::onBtClassicDeviceScanStart, this, std::placeholders::_1));
    _scanner.setDeviceUpdateCallback(std::bind(&BluetoothClassicScanner::onBtClassicDeviceUpdate, this, std::placeholders::_1));
  }

  void loop() override { _scanner.loop(); }

  void on_shutdown() override { _scanner.stop(); }

  void addDevice(const std::string &identifier, const std::string &alias) {
    esp_bd_addr_t addr;
    if (str2bda(identifier.c_str(), addr)) {
      _scanner.addKnownDevice(addr, alias.c_str());
    }
  }

 private:
  void onBtClassicDeviceScanStart(const btDeviceId_t &dev) {
    ESP_LOGI("BT-Classic", "Start scanning for device: '%s'", dev.name.c_str());
  }
  void onBtClassicDeviceUpdate(const btDeviceId_t &dev) {
    ESP_LOGI("BT-Classic", "Device update: '%s', confidence: %d", dev.name.c_str(), dev.confidence);
  }

 protected:
  BtClassicScanner _scanner;
};

}  // namespace bt_scanner
}  // namespace esphome
