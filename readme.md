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
_Proper configuration guide will be added later_
But a quick summary for now:
After you flash the ESP32 it'll create a HotSpot called 'ESP32_bt_[...]'. Connect to it using credentials __abc45678__ and configure your MQTT broker Monitor configuration, optionally the BT devices to be monitored, and as the __last step__ your WiFi credentials.
Note: When initial configuration is completed and the ESP32 is connected to your home WiFi, it is still possible to enter the configuration menu of the device by pointing a browser at the devices IP address. However, to avoid unauthorized tinkering with your precious device, any access to it after configuration is password protected using username __admin__ and password __1234__ 
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
 
