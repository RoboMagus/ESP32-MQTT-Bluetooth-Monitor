
#include <string>

#include <Arduino.h>
#include "parameter.h"
#include "mqtt.h"

#include "TelnetSerial.h"

#ifndef MQTT_CLEAN_SESSION
#define MQTT_CLEAN_SESSION          1          // 0 = No clean session, 1 = Clean session 
#endif

// Required MQTT parameters:
extern Parameter mqtt_server;
extern Parameter mqtt_port;
extern Parameter mqtt_username;
extern Parameter mqtt_password;

// Monitor arguments to be used:
// r: repeatedly scan for arrival / departure of known devices
// x: retain mqtt status messages
// b: report bluetooth beacon advertisements
// Can only effectively run in '-tad' mode!!

// Monitor MQTT preferences
// mqtt address
// mqtt broker username
// mqtt broker password
// mqtt publish topic root
// mqtt publisher identity
// mqtt port

// Behavioral preferences
// Arrival scan attempts #
// Departure scan attempts #
// Beacon expiration seconds
// min time between scans seconds
// .. others are irrelevant for now as we cannot passively scan...

// A list of max 16 known mac addresses + aliasses

// ------------------------------------------------
// Local functions:

// -----------------------------------------------
void MQTT::setup() {
  loadParams();

  // In case of a re-initialization with new parameters
  if(mqtt_client.connected()) {
    mqtt_client.disconnect();
  }

  // Setup MQTT
  if(strlen(_mqtt_server) > 1) {
    telnetSerial.printf("MQTT connecting to: %s:%s\n", _mqtt_server, _mqtt_port);
    mqtt_client.setServer(_mqtt_server, atoi(_mqtt_port));
    mqtt_client.setCallback(std::bind(&MQTT::mqtt_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
  else {
    telnetSerial.println("MQTT parameters incomplete. Not setting up MQTT.");
  }
}

// -----------------------------------------------
void MQTT::loadParams() {
  // Read MQTT parameters from storage:
  _mqtt_server   = mqtt_server.getValue();
  _mqtt_port     = mqtt_port.getValue();
  _mqtt_username = mqtt_username.getValue();
  _mqtt_password = mqtt_password.getValue();
}

// -----------------------------------------------
void MQTT::loop() {
  SCOPED_STACK_ENTRY;
  // Only loop if configured correctly
  if(!strlen(_mqtt_server)) {
    return;
  }

  if (!mqtt_client.connected()) {
      unsigned long now = millis();

      if (now - last_reconnect_attempt > 5000) {
          last_reconnect_attempt = now;

          if (reconnect()) {
              last_reconnect_attempt = 0;
          }
          else {
            telnetSerial.println("Retrying MQTT connect in 5 seconds");
          }
      }
  }
  else {
      mqtt_client.loop();
  }
}

// -----------------------------------------------
//  Used for setting service online / offline
void MQTT::setStateTopic(std::string state_topic) {
  _mqtt_state_topic_str = state_topic;
}

// -----------------------------------------------
std::string MQTT::trimWildcards(const char* topic) {
  const char* trimChars = " \t\n\r\f\v#/";
  std::string trimmed_topic = topic;
  
  trimmed_topic.erase(trimmed_topic.find_last_not_of(trimChars) + 1);

  return trimmed_topic;
}

// -----------------------------------------------
//  Send a message to a broker topic
void MQTT::send_message(const char *topic, const char *payload, bool retain)
{
    telnetSerial.printf("MQTT Outgoing on %s: ", topic);
    telnetSerial.println(payload);

    bool result = mqtt_client.publish(topic, payload, retain);

    if (!result)
    {
        telnetSerial.printf("MQTT publish to topic %s failed\n", topic);
    }
}

// -----------------------------------------------
void MQTT::add_callback(const char* topic, CallbackFunction_t callbackFunction) {
  callback_functions.push_back({topic, callbackFunction});
}

// -----------------------------------------------
void MQTT::clear_callbacks() {
  callback_functions.clear();
}

// -----------------------------------------------
void MQTT::add_subscription_topic(String topic) {
  subscription_topics.push_back(topic);
  if(mqtt_client.connected()) {
    mqtt_client.subscribe(topic.c_str());
  }
}


// -----------------------------------------------
void MQTT::remove_subscription_topic(String topic) {
  subscription_topics.erase(std::remove(subscription_topics.begin(), subscription_topics.end(), topic), subscription_topics.end());
  if(mqtt_client.connected()) {
    mqtt_client.unsubscribe(topic.c_str());
  }
}

// -----------------------------------------------
void MQTT::clear_subscription_topics() {
  subscription_topics.clear();
}

// -----------------------------------------------
void MQTT::mqtt_callback(char* topic, byte* payload, unsigned int length) {
  telnetSerial.print("Message arrived [");
  telnetSerial.print(topic);
  telnetSerial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    telnetSerial.print((char)payload[i]);
  }
  telnetSerial.println();

  for (auto& cb : callback_functions) {
    if(cb.first == topic) {
      telnetSerial.printf("Calling callback for topic '%s'\n", topic);
      cb.second(payload, length);
    }
  }
}
#if 0
// -----------------------------------------------
//  Reconnect to MQTT server and subscribe to in and out topics
bool MQTT::reconnect()
{
    // * Loop until we're reconnected
    int mqtt_reconnect_retries = 0;

    telnetSerial.println("reconnect();");
    while (!mqtt_client.connected() && mqtt_reconnect_retries < mqtt_max_reconnect_tries)
    {
        mqtt_reconnect_retries++;
        telnetSerial.printf("MQTT connection attempt %d / %d ...\n", mqtt_reconnect_retries, mqtt_max_reconnect_tries);

        // * Attempt to connect

        // if (mqtt_client.connect(_mqtt_server, _mqtt_username, _mqtt_password))
        if (mqtt_client.connect(_mqtt_server, _mqtt_username, _mqtt_password, _mqtt_state_topic_str.c_str(), 1, true, "offline", MQTT_CLEAN_SESSION))
        {
            delay(100);
            telnetSerial.println(F("MQTT connected!"));
            send_message(_mqtt_state_topic_str.c_str(), "online", true);

            delay(25);
/*
            // Subscribe to root topic
            for (auto topic : subscription_topics) {              
              bool ret = mqtt_client.subscribe(topic.c_str()); // topic may include wildcards. Needs to be trimmed for publish stuff..
              telnetSerial.printf("MQTT root topic: '%s', subscription: %d\n", topic.c_str(), ret);
              // Prevent broker disconnect:
              // mqtt_client.loop();
            }
 */           
        }
        else
        {
            telnetSerial.print(F("MQTT Connection failed: rc="));
            telnetSerial.println(mqtt_client.state());
            // telnetSerial.println(F(" Retrying in 5 seconds"));
            // telnetSerial.println("");

            return false;
            // * Wait 5 seconds before retrying
            feedLoopWDT();
            delay(5000);
        }
    }

    if (mqtt_reconnect_retries >= mqtt_max_reconnect_tries)
    {
        telnetSerial.printf("*** MQTT connection failed, giving up after %d tries ...\n", mqtt_reconnect_retries);
        return false;
    }

    return true;
}
#else
// -----------------------------------------------
//  Reconnect to MQTT server and subscribe to in and out topics
bool MQTT::reconnect()
{
    SCOPED_STACK_ENTRY;
    _mqtt_reconnect_retries++;
    telnetSerial.printf("MQTT connection attempt %d ...\n", _mqtt_reconnect_retries);

    if (mqtt_client.connect(_clientId.c_str(), _mqtt_username, _mqtt_password, _mqtt_state_topic_str.c_str(), 1, true, "offline", MQTT_CLEAN_SESSION))
    {
        delay(50);
        telnetSerial.println(F("MQTT connected!"));
        send_message(_mqtt_state_topic_str.c_str(), "online", true);

        delay(25);
        
        // Subscribe to root topic
        for (auto topic : subscription_topics) {              
          bool ret = mqtt_client.subscribe(topic.c_str()); // topic may include wildcards. Needs to be trimmed for publish stuff..
          telnetSerial.printf("MQTT root topic: '%s', subscription: %d\n", topic.c_str(), ret);
          // Prevent broker disconnect:
          mqtt_client.loop();
        }

        return true;
    }

    telnetSerial.print(F("MQTT Connection failed: rc="));
    telnetSerial.println(mqtt_client.state());

    return false;
}
#endif
