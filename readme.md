
![PlatformIO Build Status](https://img.shields.io/github/workflow/status/RoboMagus/ESP32-MQTT-Bluetooth-Monitor/Build%20PlatformIO%20Project?label=Platformio%20build&logo=github&style=for-the-badge) 
![EspHome Build Status](https://img.shields.io/github/workflow/status/RoboMagus/ESP32-MQTT-Bluetooth-Monitor/Build%20EspHome?label=EspHome%20build&logo=github&style=for-the-badge)

# ESP32 BT Monitor
![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/RoboMagus/ESP32-MQTT-Bluetooth-Monitor?include_prereleases&style=for-the-badge) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v4.4.3-red?longCache=true&style=for-the-badge)
![ESP32](https://img.shields.io/badge/ESP-32-blue?longCache=true&style=for-the-badge)

__What is it?__

This is a (at the moment __partial__) port of [andrewjfreyer/monitor](https://github.com/andrewjfreyer/monitor) for the popular and super cheap ESP32 boards. For a more detailed explanation of how it works, visit the link above. But as a short summary: this software allows you to scan for known Bluetooth devices by MAC address on demand, by requesting their name. The beauty of it is that you don't need any special software running on the devices you'd like to track!

__iBeacon support:__

Currently, support for ble iBeacon devices is also being implemented! 
The ESP32 will in the background continually listen passively to broadcasts of iBeacon devices. If a found ble __UUID__ matches one of the configured __UUID__'s then an MQTT message will be broadcast that includes the _RSSI_ of the found service.
_Note_ that this feature is currently still subject to change!!


## Building the project from source
### Prerequisites
In order to build this project, you need to have __Visual Studio Code__ installed, with the __C/C++__ and __PlatformIO IDE__ extensions.

### Libraries
This project relies on the following libraries:
- [ropg/ezTime](https://github.com/ropg/ezTime)
- [knolleary/PubSubClient](https://github.com/knolleary/PubSubClient#v2.8)
- [RoboMagus/WiFiManager](https://github.com/RoboMagus/WiFiManager) (Modified from [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager))

### Build it
1. Clone this repository
2. Open the local repository directory in VSCode
3. Open the PIO _Projects & Configuration_ tab and create an existing project.
4. Open this project from the PIO interface
5. In the sidebar, open up the PIO Tab and under _PROJECT TASKS_ / _Default_ hit _Build_
	Note: The first time opening this project, PlatformIO will take some time to fetch all the required modules before the list under _PROJECT TASKS_ shows up!!
6. Once the build succeeds you can flash your ESP32 with the '_Monitor_' compatible Bluetooth scanner.

__Modifications:__
When building the project from sources, there are some things you may want to customize in ```platformio.ini```.
- ```AP_PASSWD```: The AccessPoint password for when the ESP32 is not yet configured to connect to your home WiFi network.
- ```HTTPD_USER```: Username for authorization on the config portal when the ESP32 is connected to your home WiFi network.
- ```HTTPD_PASSWD```: Password for authorization on the config portal when the ESP32 is connected to your home WiFi network.
- ```MAX_NUM_STORED_BLUETOOTH_DEVICES```: Maximum number of _known bluetooth devices_ that can be stored on the ESP32

By default PlatformIO is set up for flashing devices that are connected to your computer. If you wish to perform remote debugging or over the air updates using ```espota``` you'll need to uncomment the following section and fill in your device-ip:
```
monitor_port = socket://[device-ip]:23
upload_protocol = espota
upload_port = [device-ip]
upload_flags =
  --port=3232
  --auth=admin
```


## Configuration
After flasing your ESP32 some initial configuration is required. To begin, connect your phone to the ESP32's WiFi hotspot. This will have a name starting with ```ESP32_bt```, followed by unique characters based on the devices MAC address. The default firmware configuration requires the following credentials in order to connect to the AP: ```abc45678```. When connected navigate with your browser of choice to ```192.168.4.1``` to open the configuration menu.

<img src="/doc/screenshots/main_menu.jpg" width=480px>

Now it's best to first enter the __settings__ menu to start configuration there. Performing the WiFi configuration first will cause the device to restart and connect to your home network. You can still configure the device after this, it just requires some more steps to navigate to it's new IP again!
The settings menu contains all settings related to the BT monitor application as shown below. The scan settings and BT devices can be skipped and configured later on. The important bits for now are the MQTT parameters to make sure the application can communicate.

<img src="/doc/screenshots/parameter_page.jpg" width=480px>

In case you need to revisit the configuration pages you can do so by navigating to the ESP32's IP address on your home network. To prevent others from tinkering with your device any access to it's webpage once it's connected to your WiFi network is password protected! To log on, use username ```admin``` and password ```1234```.

## Usage
### web control
The ESP32 bluetooth scan status can be seen through the devices webpage by following the __BT Monitor__ link in the main menu. This page will show you all configured devices by name and MAC address, as well as their away / present state. From this page you can also trigger various scans and update the page to load the new scan results.

<img src="/doc/screenshots/bt_monitor_page.png" width=480px>

### Home assistant
Find some examples for sensors and automations in HomeAssistant [here](doc/HomeAssistantSnippets.md).

## Features
### Implemented:
- [andrewjfreyer/monitor](https://github.com/andrewjfreyer/monitor) compatible MQTT scan reporting		
- Arrival / Departure scan triggered by ```monitor/scan/arrive``` and ```monitor/scan/depart```
  - 'Any' scan (scan for departure on present devices and arrival for away devices) triggered by ```monitor/scan/any```
- Periodic scanning
- Setup known bluetooth devices through MQTT using ```monitor/setup/add known device``` and ```monitor/setup/delete known device```
  - Send only MAC when deleting a device
  - Send MAC followed by its alias when adding devices
- Over the air update
  - through web interface
  - Using ```espota```
- Setup through [modified](https://github.com/RoboMagus/WiFiManager) [WiFiManager](https://github.com/tzapu/WiFiManager)
  - Password protected web interface once connected to your home WiFi network
  - Bluetooth Scan status + scan controls webpage
- Logging over serial and telnet
- Monitor settings:
  - #Arrival scans
  - #Departure scans
  - seconds between scans within set
  - minimal time between scan sets
  - periodic scanning interval
  
### ToDo
- [ ] Add passive scanning as is available in [monitor](https://github.com/andrewjfreyer/monitor)
- [ ] Expand configuration options
- [ ] Add BLE iBeacon scanning support for room by room presence detection
  - [x] Initial support __(This is subject to change!!)__
- [ ] Add support for interacting with ble keyfinders
- [ ] Strip away the core BT monitor logic to expose as a library
- [ ] Make an ESPHome component for it
	- **Work In Progress**
 
