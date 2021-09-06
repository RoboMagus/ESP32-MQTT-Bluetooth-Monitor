// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <initializer_list>
// IDF headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include "time.h"

#include <ezTime.h>

#include "esp_task_wdt.h"

// Custom components:
#include "NestedStruct.h"
#include "TelnetSerial.h"
#include "WiFiComponent.h"
#include "BluetoothScanner.h"
#include "parameter.h"
#include "BluetoothParameter.h"
#include "led.h"
#include "mqtt.h"

#include "stackDbgHelper.h"

#ifndef MAX_NUM_STORED_BLUETOOTH_DEVICES
#define MAX_NUM_STORED_BLUETOOTH_DEVICES (8)
#endif
// Upper limit for max number of stored bluetooth devices. 
// Is rediculously large, but more would not fit in NVS!!
static_assert(MAX_NUM_STORED_BLUETOOTH_DEVICES <= 40, "Cannot fit more than 40 bluetooth device identifiers in ESP32 storage!!");

#define WDT_TIMEOUT 8

// ================================================
// >>>>>>>>>>  Global objects  >>>>>>>>>>>>>>>>>>>>

#ifndef NO_USE_TELNET_SERIAL
TelnetSerial telnetSerial;
Stream& mSerial = telnetSerial;
#else
Stream& mSerial = Serial;
#endif
Preferences preferences;
WiFiManager wm(mSerial);
LED led;

// >>> All parameters in order shown in WiFiManager:
Parameter time_header  ("<h3>Timezone</h3><br>Use e.g. Europe/Amsterdam. <a href='http://wikipedia.org/wiki/List_of_tz_database_time_zones#list'>List of timezones</a><br>(empty for auto-geolocation)<br>");
Parameter time_zone    ("time_zone",     "",            "",         40);
// MQTT
Parameter mqtt_header  ("<h3>MQTT parameters</h3>");
Parameter mqtt_server  ("mqtt_server",   "mqtt server", "",         40);
Parameter mqtt_port    ("mqtt_port",     "mqtt port",   "1883",      6);
Parameter mqtt_username("mqtt_username", "username",    "",         32);
Parameter mqtt_password("mqtt_password", "password",    "",         32);

Parameter mqtt_topic   ("mqtt_topic",    "topic root",  "monitor",  32);
Parameter mqtt_identity("mqtt_identity", "identity",    "",         32);

// Bluetooth Monitor settings
Parameter bluetooth_monitor_header                     (PSTR("<h3>Bluetooth Monitor settings</h3>"));
Parameter bluetooth_monitor_arrival_scans              (PSTR("bm_arrival"),    PSTR("# Arrival scans"),               "1",     6);
Parameter bluetooth_monitor_departure_scans            (PSTR("bm_depart"),     PSTR("# Departure scans"),             "3",     6);
Parameter bluetooth_monitor_seconds_between_scan_iters (PSTR("bm_iter_time"),  PSTR("Seconds between scan tries"),    "3",     6);
Parameter bluetooth_monitor_beacon_expiration          (PSTR("bm_beacon_exp"), PSTR("Beacon expiration time (s)"),    "240",   6);
Parameter bluetooth_monitor_min_time_between_scans     (PSTR("bm_min_time"),   PSTR("Min. time between scans (s)"),   "10",    6);
Parameter bluetooth_monitor_periodic_scan_interval     (PSTR("bm_period"),     PSTR("Periodic scan interval (s)<br>(Leave empty to disable periodic scanning)"),    "",      6);

// Known Static Bluetooth MAC 
// Nested struct helper to generate objects and reduce boilerplate...
NestWrapper<BluetoothParameter, MAX_NUM_STORED_BLUETOOTH_DEVICES> bluetooth_monitor_parameter_sets;

// <<<
WiFiComponent wifi(mSerial);
BluetoothScanner btScanner(mSerial);
MQTT mqtt(mSerial);

Timezone mTime;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


std::stack<stack_entry> _stack;

uint32_t startFreeHeap = 0;

#define USE_SEPARATE_STATS_TASK 0

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_task_info.h"

#define MAX_TASK_NUM 20                         // Max number of per tasks info that it can store
#define MAX_BLOCK_NUM 20                        // Max number of per block info that it can store

static size_t s_prepopulated_num = 0;
static heap_task_totals_t s_totals_arr[MAX_TASK_NUM];
static heap_task_block_t s_block_arr[MAX_BLOCK_NUM];

// -----------------------------------------------
extern "C" void esp_task_wdt_isr_user_handler(void)
{
    printf("\nSTACK:\n");
    while(!_stack.empty()) {
        auto& e = _stack.top();
        printf("PC: %d, Func: %s, Line: %d \n", e.pc, e.func, e.line);
        _stack.pop();
    }
}

// -----------------------------------------------
static void esp_dump_per_task_heap_info(void)
{
    SCOPED_STACK_ENTRY;
    heap_task_info_params_t heap_info = {0};
    heap_info.caps[0] = MALLOC_CAP_8BIT;        // Gets heap with CAP_8BIT capabilities
    heap_info.mask[0] = MALLOC_CAP_8BIT;
    heap_info.caps[1] = MALLOC_CAP_32BIT;       // Gets heap info with CAP_32BIT capabilities
    heap_info.mask[1] = MALLOC_CAP_32BIT;
    heap_info.tasks = NULL;                     // Passing NULL captures heap info for all tasks
    heap_info.num_tasks = 0;
    heap_info.totals = s_totals_arr;            // Gets task wise allocation details
    heap_info.num_totals = &s_prepopulated_num;
    heap_info.max_totals = MAX_TASK_NUM;        // Maximum length of "s_totals_arr"
    heap_info.blocks = s_block_arr;             // Gets block wise allocation details. For each block, gets owner task, address and size
    heap_info.max_blocks = MAX_BLOCK_NUM;       // Maximum length of "s_block_arr"

    heap_caps_get_per_task_info(&heap_info);

    for (int i = 0 ; i < *heap_info.num_totals; i++) {
        const char* taskName = heap_info.totals[i].task ? pcTaskGetTaskName(heap_info.totals[i].task) : nullptr;
        bool tryHeap = true;
        if(taskName == nullptr) {
            tryHeap = false;
        }
        else {
            for (int i = 0; i < strlen(taskName); i++) {
                if(!isascii(taskName[i])) {
                    tryHeap = false;
                    break;
                }
            }
        }
        
        mSerial.printf("Task: %22s -> CAP_8BIT: %6d CAP_32BIT: %3d -> HighWaterMark: %5d\n",
                heap_info.totals[i].task ? pcTaskGetTaskName(heap_info.totals[i].task) : "Pre-Scheduler allocs" ,
                heap_info.totals[i].size[0],    // Heap size with CAP_8BIT capabilities
                heap_info.totals[i].size[1],    // Heap size with CAP32_BIT capabilities
                tryHeap ? uxTaskGetStackHighWaterMark(heap_info.totals[i].task) : -1);   
                
        feedLoopWDT(); // Must be done <= 360 mS
    }

    mSerial.printf("Free heap at start: %d, min free heap ever: %d\n", startFreeHeap, xPortGetMinimumEverFreeHeapSize());
    mSerial.printf("Free heap size: %d, largest fee block: %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    mSerial.print("HeapCapsFreeSize - Internal: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    mSerial.print(" SPIRAM: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    mSerial.print(" IRAM8: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT));
    mSerial.print(" Default: ");
    mSerial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    mSerial.printf("\n\n");
}

// -----------------------------------------------
void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
    // No Printf to avoid MALLOCS!!!
    Serial.print(function_name);
    Serial.print(" in task ");
    Serial.print(pcTaskGetTaskName(NULL));
    Serial.print(" was called but failed to allocate ");
    Serial.print(requested_size);
    Serial.print(" bytes, with capabilities: ");
    Serial.println(caps);
    Serial.print(" .High water mark: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    Serial.print("HeapCapsFreeSize - Internal: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.print(" SPIRAM: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.print(" IRAM8: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT));
    Serial.print(" Default: ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    
    Serial.print("Free heap at start: ");
    Serial.print(startFreeHeap);
    Serial.print(", min free heap ever: ");
    Serial.println(xPortGetMinimumEverFreeHeapSize());
    Serial.print("Free heap size: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.print(", largest fee block: ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    //Serial.printf("%s in task '%s' was called but failed to allocate %d bytes with 0x%X capabilities. HighWaterMark: %lu \n",function_name, pcTaskGetTaskName(NULL), requested_size, caps, (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

// -----------------------------------------------
#define PREFERENCES_TAG_VALUE (54)
void setupPreferences() {
    preferences.begin("E32BT", false);
    
    // Perform a factory reset if we have not ran before...
    if(preferences.getUInt("ESP32_BT_TAG", 0) != PREFERENCES_TAG_VALUE) {
        Serial.print("Could not find Magic value in preferences. Clearing NVS for factory reset!");

        preferences.end();

        nvs_flash_erase(); // erase the NVS partition and...
        nvs_flash_init(); // initialize the NVS partition.
        Serial.print(" ... ");

        preferences.begin("E32BT", false);
        preferences.putUInt("ESP32_BT_TAG", PREFERENCES_TAG_VALUE);
        Serial.println("Done!!");
    }

    // Remove all preferences under the opened namespace
    //preferences.clear();

    // Or remove the counter key only
    //preferences.remove("counter");
}

// -----------------------------------------------
void setupMQTT() {
    mqtt.setup( mqtt_server.getValue(),
                mqtt_port.getValue(),
                mqtt_username.getValue(),
                mqtt_password.getValue() );
}

// -----------------------------------------------
void setupMqttCallbacks() {
    // Clear existing topics and callbacks based on old parameters:
    mqtt.clear_subscription_topics();
    mqtt.clear_callbacks();

    std::string baseTopic = mqtt.trimWildcards(mqtt_topic.getValue());
    mqtt.setStateTopic(baseTopic + "/" + mqtt_identity.getValue() + "/status");
    // ToDo: this becomes a problem when reloading parameters at runtime!!
    // FIX THIS!
    //mqtt.add_subscription_topic(mqtt_topic.getValue()); //TEST
    mqtt.add_subscription_topic((baseTopic + "/scan/#").c_str());
    mqtt.add_subscription_topic((baseTopic + "/setup/#").c_str());
    mqtt.add_callback((baseTopic + "/scan/ARRIVE").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Arrival);
    });
    mqtt.add_callback((baseTopic + "/scan/DEPART").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Departure);
    });
    mqtt.add_callback((baseTopic + "/scan/ANY").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Either);
    });
    // MQTT setup interfaces:
    mqtt.add_callback((baseTopic + "/setup/ADD STATIC DEVICE").c_str(), [](const byte* payload, unsigned int length) {
        // Splitting the MAC from the alias:
        std::string _in = std::string((const char*)payload, length);
        _in.erase(_in.find_last_not_of(" \t\n\v") + 1);
        std::string mac = _in.substr(0, _in.find(" "));
        std::string alias = _in.substr(_in.find(" "));

        size_t first = alias.find_first_not_of(" \t\n\v");
        size_t last = alias.find_last_not_of(" \t\n\v");
        if(first != std::string::npos) {
            alias = alias.substr(first, (last-first+1));
        }

        // Add to storage and BluetoothScanner
        esp_bd_addr_t addr;
        if(str2bda(mac.c_str(), addr)) {
            char bda_str[18];
            if (bda2str(addr, bda_str, 18)) {
                bool device_saved = false;
                for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
                    if(strlen(bluetooth_monitor_parameter_sets.data[i].getMacAddress()) == 0 || strcmp(bluetooth_monitor_parameter_sets.data[i].getMacAddress(), bda_str) == 0) {
                        bluetooth_monitor_parameter_sets.data[i].setMacAddress(bda_str);
                        bluetooth_monitor_parameter_sets.data[i].setAlias(alias.c_str());
                        device_saved = true;
                        break;
                    }
                }
                if (device_saved){
                    btScanner.addKnownDevice(addr, alias.c_str());
                }
            }
        }

    });
    mqtt.add_callback((baseTopic + "/setup/DELETE STATIC DEVICE").c_str(), [](const byte* payload, unsigned int length) {
        esp_bd_addr_t addr;
        if(str2bda(std::string((const char*)payload, length).c_str(), addr)) {
            char bda_str[18];
            if (bda2str(addr, bda_str, 18)) {
                btScanner.deleteKnownDevice(addr);
                for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
                    if(strcmp(bluetooth_monitor_parameter_sets.data[i].getMacAddress(), bda_str) == 0) {
                        bluetooth_monitor_parameter_sets.data[i].setMacAddress("");
                        bluetooth_monitor_parameter_sets.data[i].setAlias("");
                    }
                }
            }
        }
    });
}

// -----------------------------------------------
void setupTimeZone() {    
	// Or country codes for countries that do not span multiple timezones
	//mTime.setLocation(F("nl"));
    const char* timezone = time_zone.getValue();
	mTime.setLocation(strlen(timezone) ? timezone : "GeoIP");
	Serial.print(F("Local time:         "));
	Serial.println(mTime.dateTime());
}


#if CONFIG_AUTOSTART_ARDUINO
// -----------------------------------------------
void setup() {
    startFreeHeap = xPortGetFreeHeapSize();
    esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

    // Turn on LED at start of setup
    led.setup();
    led.set(ON);
     
    WiFi.setAutoConnect(true);

    // Begin Serial separate!
    Serial.begin(115200, SERIAL_8N1);

    setupPreferences();

    mSerial.println("Booting");
    ezt::setDebug(INFO, mSerial);

    if(error != ESP_OK) {
        Serial.printf("failed alloc callback error: %d\n", error);
    }

    // Note that this will not show up in the MENU!!!
    wifi.addCustomHtmlPage("/bt", [](){
        String page = FPSTR(HTTP_HEAD_START);
        page.replace(FPSTR(T_v), "Bluetooth Status");
        page += FPSTR(HTTP_SCRIPT);
        page += FPSTR(HTTP_STYLE);
        // Selection for DarkMode: _bodyClass = enable ? "invert" : "";
        String p = FPSTR(HTTP_HEAD_END);
        p.replace(FPSTR(T_c), "invert"); // add class str
        page += p;

        auto& devList = btScanner.getBtDeviceStates();
        char bda_str[18];
        for (auto& dev : devList) {
            String str = FPSTR("<div class='msg {C}'><strong>{n}</strong> is {s}<br/><em><small>{m}</small></em></div>");
            str.replace("{C}", (dev.state == 1 ? "S" : "D"));
            str.replace("{n}", dev.name);
            str.replace("{s}", (dev.state == 1 ? "Present" : "Away"));
            bda2str(dev.mac, bda_str, 18);
            str.replace("{m}", bda_str);
            page += str;
        }

        page += FPSTR("<hr><br/>"  // MENU_SEP
                      "<br/><form action='/bt?arrive=1' method='POST'><button name='arrival' value='1'>Arrival Scan</button></form>"
                      "<br/><form action='/bt?depart=1' method='POST'><button name='departure' value='1'>Departure Scan</button></form>"
                      "<br/><form action='/bt?scan=1' method='POST'><button name='scan' value='1'>'Any' Scan</button></form>"
                      "<br/><form action='/bt?refresh=1' method='POST'><button name='refresh' value='1'>Refresh</button></form>");

        if(wm.server->hasArg(F("arrive"))) {
            btScanner.startBluetoothScan(ScanType::Arrival);
        }
        if(wm.server->hasArg(F("depart"))) {
            btScanner.startBluetoothScan(ScanType::Departure);
        }
        if(wm.server->hasArg(F("scan"))) {
            btScanner.startBluetoothScan(ScanType::Either);
        }

        page += FPSTR(HTTP_END);

        wm.server->send(200, FPSTR(HTTP_HEAD_CT), page);        
    });

    // Blocking if not connected WiFi AP
    wifi.setup();
    // Stop bluetooth on OTA to reduce chance of failures..
    wifi.registerOtaStartCallback([](){
        mSerial.println("Stopping BtScanner for OTA...");
        btScanner.stop();
    });

#ifndef NO_USE_TELNET_SERIAL
    telnetSerial.begin(); // Start the telnet bit...
#endif
    setupMQTT();
    setupMqttCallbacks();

    // Setup bluetooth AFTER WiFi such that the persistent parameters are loaded!
    btScanner.init();
    btScanner.setup();
    for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
        esp_bd_addr_t mac;
        // If valid mac address, add it!
        const char* mac_str = bluetooth_monitor_parameter_sets.data[i].getMacAddress();
        mSerial.printf("Validating MAC: %s \n", mac_str);
        if(str2bda(mac_str, mac)) {
            const char* alias = bluetooth_monitor_parameter_sets.data[i].getAlias();
            mSerial.printf("  Adding device: %s, with MAC: %s \n", alias, mac_str);
            btScanner.addKnownDevice(mac, alias);
        }
    }

    // EZTime: sync time
	ezt::waitForSync();
    setupTimeZone();

    // Stop bluetooth on OTA to reduce chance of failures..
    wifi.registerOtaStartCallback([](){
        mSerial.println("Stopping BtScanner for OTA...");
        btScanner.stop();
    });
    // Run MQTT setup again when new parameters are made available
    wifi.registerParamSaveCallback(setupMQTT); // This causes pubSub to destruct and panic!!
    wifi.registerParamSaveCallback(setupMqttCallbacks);
    wifi.registerParamSaveCallback(setupTimeZone);
    // ToDo: Reload of Bluetooth parameters

    // Turn off led at the end of setup
    led.set(OFF);

    btScanner.startBluetoothScan(ScanType::Either);

    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts

    enableLoopWDT();
}

#if MEM_DEBUG_DUMP_ENABLED
unsigned long last_millis = 0;
unsigned long interval=10000; // 10sec
uint16_t count = 0;
#endif

// -----------------------------------------------
void loop() {
    SCOPED_STACK_ENTRY;

    wifi.loop();
#ifndef NO_USE_TELNET_SERIAL
    telnetSerial.loop();
#endif

    mqtt.loop();
    delay(10);
    btScanner.loop();

#if MEM_DEBUG_DUMP_ENABLED
    unsigned long current_millis = millis();
    if(current_millis > last_millis + interval) {
        last_millis = current_millis;
        mSerial.println(current_millis);
        
	    mSerial.println(mTime.dateTime());
        esp_dump_per_task_heap_info();

        count++;
    }    
#endif

    // WDT Feeding is done in Arduino scope outside of the loop...
	// Delay needed to give idle task some room!!
    delay(10);
}

#else
#ERROR wrong SDK configuration!
#endif 