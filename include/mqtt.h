#ifndef MQTT_H
#define MQTT_H

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <utility>

// Arduino headers
#include <Arduino.h>
#include <PubSubClient.h>
#include "WiFiClient.h"


class MQTT {
public:
    typedef std::function<void(const byte*, unsigned int)> CallbackFunction_t;

    MQTT() : mqtt_client(espClient)
    {
        _clientId = String("ESP32_bt") + String(WIFI_getChipId(),HEX);
        // Nothing..
    }

    void setup();
    void loop();

    void setStateTopic(std::string state_topic);
    static std::string trimWildcards(const char* topic);

    void send_message(const char *topic, const char *payload, bool retain = false);

    void add_callback(const char* topic, CallbackFunction_t callbackFunction);
    void clear_callbacks();
    
    void add_subscription_topic(String topic);
    void remove_subscription_topic(String topic);
    void clear_subscription_topics();

private:

    void loadParams();

    void mqtt_callback(char* topic, byte* payload, unsigned int length);
    bool reconnect();

    WiFiClient espClient;
    PubSubClient mqtt_client;

    // Stored MQTT parameters 
    const char* _mqtt_server   = nullptr;
    const char* _mqtt_port     = nullptr;
    const char* _mqtt_username = nullptr;
    const char* _mqtt_password = nullptr;

    uint32_t _mqtt_reconnect_retries = 0;

    std::string _mqtt_state_topic_str;
    String _clientId;

    // MQTT Last reconnection counter
    unsigned long last_reconnect_attempt = 0;

    std::vector<std::pair<String, CallbackFunction_t>> callback_functions;
    std::vector<String> subscription_topics;

    static const uint8_t mqtt_max_reconnect_tries = 10;
};


extern MQTT mqtt;

#endif // MQTT_H
