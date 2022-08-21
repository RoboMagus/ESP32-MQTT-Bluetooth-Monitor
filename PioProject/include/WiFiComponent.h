#ifndef WIFICOMPONENT_h
#define WIFICOMPONENT_h

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <utility>

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// Configuration access point password:
#ifndef AP_PASSWD
#define AP_PASSWD "abc45678"
#endif

// Config portal httpd auth:
#ifndef HTTPD_USER
#define HTTPD_USER "admin"
#endif
#ifndef HTTPD_PASSWD
#define HTTPD_PASSWD "1234"
#endif

class WiFiComponent {
public:
    typedef std::function<void(void)> customHtmlHandler_t;

    WiFiComponent(Stream& serialStream) : mSerial(serialStream) 
    {
        // Nothing
    }

    void setup();
    void loop();

    void registerOtaStartCallback(std::function<void(void)> callback);

    void registerParamSaveCallback(std::function<void(void)> callback);

    // Note: This must be called before 'setup()'!!
    void addCustomHtmlPage(const char* path, customHtmlHandler_t handler);

private:
    void wifiInfo();

    // Setup functions
    void setupArduinoOTA();
    void setupConfigPortal();

    void stopArduinoOTA();

    // Parameter handling functions
    void loadParams();
    void storeParams();

    // Callbacks
    void saveWifiCallback();
    void configModeCallback (WiFiManager *myWiFiManager);
    void saveParamCallback();
    void handleRoute();
    void bindServerCallback();

    Stream& mSerial;

    uint8_t otaBusy = 0;
    std::vector<std::function<void(void)>> ota_callback_functions;

    std::vector<std::function<void(void)>> paramSaved_callback_functions;

    struct customPage_t {
        const char* path;
        customHtmlHandler_t handler;

        customPage_t(const char* Path, customHtmlHandler_t Handler) : path(Path), handler(Handler) {
        }
    };
    std::vector<customPage_t> custom_html_page_handlers;
    String _hostname;
};

#endif // WIFICOMPONENT_h