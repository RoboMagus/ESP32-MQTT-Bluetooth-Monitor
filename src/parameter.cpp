// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Tweaked SDK configuration
#include "parameter.h"

// Arduino includes
#include <Arduino.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Preferences.h>

// Required available global entities:
extern WiFiManager wm;
extern Preferences preferences;

// Free functions:
// -----------------------------------------------
bool param2bool(const Parameter& param) {
    String val = param.getValue();
    val.toLowerCase();

    if(val == "true" || val == "1") {
        return true;
    }

    // fallback case: always false
    return false;
}


// Parameter class functions:

// -----------------------------------------------
Parameter::Parameter(const char *custom) : WiFiManagerParameter(custom)  
{
    // Automatically add parameter to WiFiManager upon creation.
    wm.addParameter(this);
}

// -----------------------------------------------
Parameter::Parameter(const char *id, const char *label, const char* defaultValue, int length, const char *custom, int labelPlacement) 
    : WiFiManagerParameter(id, label, defaultValue, length, custom, labelPlacement) 
{
    // Automatically add parameter to WiFiManager upon creation.
    wm.addParameter(this);
}

// -----------------------------------------------
void Parameter::initialize() {
    // CustomHTLM does not store to NVS. No need to read it...
    if(getID() != nullptr) {
        preferences.getString(WiFiManagerParameter::getID(), WiFiManagerParameter::_value, WiFiManagerParameter::_length);
    }
    _initialized = true;
}

// -----------------------------------------------
void Parameter::storeValue() {
    // CustomHTLM does not store to NVS. No need to store it...
    if(getID() != nullptr) {
        if(preferences.getString(WiFiManagerParameter::getID(),"") != WiFiManagerParameter::_value) {
            preferences.putString(WiFiManagerParameter::getID(), getValue());
        }
    }
}

// -----------------------------------------------
void Parameter::setValue(const char *value) {
    if(_initialized) {
        preferences.putString(WiFiManagerParameter::getID(), value);
    }
    WiFiManagerParameter::setValue(value);
}
