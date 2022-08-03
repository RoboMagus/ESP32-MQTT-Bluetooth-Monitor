#ifndef Parameter_h
#define Parameter_h

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Tweaked SDK configuration
#include "sdkconfig.h"

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager


#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAX(a ,b) ((a) > (b) ? (a) : (b))


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

template<class T, typename std::enable_if<std::is_integral<T>::value, int>::type* = nullptr>
constexpr int maxNumDigits(const T maxVal) {
    return MAX(6, static_cast<int>(log10(abs(maxVal))) + 2);
}

template<class T, typename std::enable_if<std::is_floating_point<T>::value, int>::type* = nullptr>
constexpr int maxNumDigits(const T maxVal) {
    return 12;
}

// Numeric parameter:
template<class T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class NumericParameter : public Parameter
{
public:
    NumericParameter(const char *id, const char *label, T defaultValue, T minValue = std::numeric_limits<T>::min(), T maxValue = std::numeric_limits<T>::max(), int labelPlacement = WFM_LABEL_BEFORE) : 
        Parameter(id, label, nullptr, maxNumDigits(std::numeric_limits<T>::max()), nullptr, labelPlacement), m_min(minValue), m_max(maxValue) 
    {
        std::string strVal = std::to_string(defaultValue);
        WiFiManagerParameter::setValue(strVal.c_str());
    }

    void setValue(const T value) {
        std::string strVal = std::to_string(value);
        Parameter::setValue(strVal.c_str());
    }

    template<class Q = T>
    const typename std::enable_if<std::is_floating_point<Q>::value, T>::type getValue() {
        T val = atof(Parameter::getValue());
        T cVal = CLAMP(val, m_min, m_max);
        if (cVal != val) {
            setValue(cVal);
        }
        return cVal;
    }

    template<class Q = T>
    const typename std::enable_if<std::is_integral<Q>::value, T>::type getValue() {
        long long val = atoll(Parameter::getValue());
        T tVal = CLAMP(val, m_min, m_max);
        if (tVal != val) {
            setValue(tVal);
        }
        return tVal;
    }

    operator T() {
        return getValue();
    }

    T& operator=(T value) {
        setValue(value);
        return value;
    }

private:
    const T m_min;
    const T m_max;
};

typedef NumericParameter<bool>           BoolParameter;
typedef NumericParameter<uint8_t>        U8Parameter;
typedef NumericParameter< int8_t>        I8Parameter;
typedef NumericParameter<uint16_t>       U16Parameter;
typedef NumericParameter< int16_t>       I16Parameter;
typedef NumericParameter<uint32_t>       U32Parameter;
typedef NumericParameter< int32_t>       I32Parameter;
typedef NumericParameter<int>            IntParameter;
typedef NumericParameter<long>           LongParameter;
typedef NumericParameter<unsigned long>  ULongParameter;
typedef NumericParameter<float>          FloatParameter;

#endif // Parameter_h
