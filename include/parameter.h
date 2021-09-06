#ifndef Parameter_h
#define Parameter_h

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Tweaked SDK configuration
#include "sdkconfig.h"

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager


class Parameter : public WiFiManagerParameter{
public:
    // Constructors for different Parameter types:
    Parameter(const char *custom);
    Parameter(const char *id, const char *label, const char* defaultValue = "", int length = 0, const char *custom = nullptr, int labelPlacement = WFM_LABEL_BEFORE);

    void initialize();
    void storeValue();

    void setValue(const char *value);

private:
    bool _initialized = false;
};

bool param2bool(const Parameter& param);

#endif // Parameter_h
