#ifndef Parameter_h
#define Parameter_h

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Preferences.h>

// Required available global entities:
extern WiFiManager wm;
extern Preferences preferences;


class Parameter : public WiFiManagerParameter{
public:
    Parameter(const char *custom) : WiFiManagerParameter(custom)  
    {
        // Automatically add parameter to WiFiManager upon creation.
        wm.addParameter(this);
    }

    Parameter(const char *id, const char *label, const char* defaultValue = "", int length = 0, const char *custom = nullptr, int labelPlacement = WFM_LABEL_BEFORE) 
        : WiFiManagerParameter(id, label, defaultValue, length, custom, labelPlacement) 
    {
        // Automatically add parameter to WiFiManager upon creation.
        wm.addParameter(this);
    }

    void initialize() {
        // CustomHTLM does not store to NVS. No need to read it...
        if(getID() != nullptr) {
            preferences.getString(WiFiManagerParameter::getID(), WiFiManagerParameter::_value, WiFiManagerParameter::_length);
        }
        _initialized = true;
    }

    void storeValue() {
        // CustomHTLM does not store to NVS. No need to store it...
        if(getID() != nullptr) {
            if(preferences.getString(WiFiManagerParameter::getID(),"") != WiFiManagerParameter::_value) {
             preferences.putString(WiFiManagerParameter::getID(), getValue());
            }
        }
    }

    void setValue(const char *value) {
        if(_initialized) {
            preferences.putString(WiFiManagerParameter::getID(), value);
        }
        WiFiManagerParameter::setValue(value);
    }

private:
    bool _initialized = false;
};


#endif // Parameter_h
