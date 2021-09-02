# ESP32 BT Monitor


## Building the project from source
### Prerequisites
In order to build this project, you need to have __Visual Studio Code__ installed, with the __C/C++__ and __PlatformIO IDE__ extensions.

### Build it
1. Clone this repository
2. Open the local repository directory in VSCode
3. Open the PIO _Projects & Configuration_ tab and create an existing project.
4. Open this project from the PIO interface
5. In the sidebar, open up the PIO Tab and under _PROJECT TASKS_ / _Default_ hit _Build_
	Note: The first time opening this project, PlatformIO will take some time to fetch all the required modules before the list under _PROJECT TASKS_ shows up!!
6. Once the build succeeds you can flash your ESP32 with the '_Monitor_' compatible Bluetooth scanner.

## Configuration
After flasing your ESP32 some initial configuration is required. To begin, connect your phone to the ESP32's WiFi hotspot. This will have a name starting with ```ESP32_bt```, followed by unique characters based on the devices MAC address. The default firmware configuration requires the following credentials in order to connect to the AP: ```abc45678```. When connected navigate with your browser of choice to ```192.168.4.1``` to open the configuration menu.
![main menu](/doc/screenshots/main_menu.jpg)

Now it's best to first enter the __settings__ menu to start configuration there. Performing the WiFi configuration first will cause the device to restart and connect to your home network. You can still configure the device after this, it just requires some more steps to navigate to it's new IP again!
The settings menu contains all settings related to the BT monitor application as shown below. The scan settings and BT devices can be skipped and configured later on. The important bits for now are the MQTT parameters to make sure the application can communicate.
![main menu](/doc/screenshots/parameter_page.jpg)

In case you need to revisit the configuration pages you can do so by navigating to the ESP32's IP address on your home network. To prevent others from tinkering with your device any access to it's webpage once it's connected to your WiFi network is password protected! To log on, use username ```admin``` and password ```1234```.

## Usage
### web control
The ESP32 bluetooth scan status can be seen through the devices webpage by following the __BT Monitor__ link in the main menu. This page will show you all configured devices by name and MAC address, as well as their away / present state. From this page you can also trigger various scans and update the page to load the new scan results.
![main menu](/doc/screenshots/bt_monitor_page.png)

### Home assistant
__todo__

## Features
### Implemented:
will be added later...

### ToDo
- [ ] Add passive scanning as is available in [monitor](https://github.com/andrewjfreyer/monitor)
- [ ] Expand configuration options
- [ ] Add BLE iBeacon scanning support for room by room presence detection
- [ ] Strip away the core BT monitor logic to expose as a library
- [ ] Make an ESPHome component for it
	- Currently blocked by incompatible IDF v4.2 support used by ESPHome Core
 
