/*
 TelnetSerial.h - Arduino like serial library for printing over Telnet for ESP8266 & ESP32
 */

#ifndef TelnetSerial_h
#define TelnetSerial_h

#include <inttypes.h>
#include <../include/time.h> // See issue #6714
#include "Stream.h"
#include "WiFiServer.h"
#include "WiFiClient.h"

#include "stackDbgHelper.h"

#define MAX_TELNET_CLIENTS  2

class Telnet: public Stream
{
public:
    Telnet(uint16_t port = 23);
    virtual ~Telnet() {}

    void begin();

    void end();

    void handleConnections();

    int available(void) override;

    void flush(void) override;

    int peek(void) override
    {
        int return_value = -1;
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                return_value = serverClients[i].peek();
                if (return_value >= 0) {
                    break;
                }
            }
        }
        return return_value;
    }

    int read(void) override
    {
        int return_value = -1;
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                return_value = serverClients[i].read();
                if (return_value >= 0) {
                    break;
                }
            }
        }
        return return_value;
    }
    
    size_t read(char* buffer, size_t size)
    {
        size_t return_value = 0;
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                return_value = serverClients[i].read((uint8_t*)buffer, size);
                if (return_value > 0) {
                    break;
                }
            }
        }
        return return_value;
    }
    size_t readBytes(char* buffer, size_t size) override
    {
        size_t return_value = 0;
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                return_value = serverClients[i].read((uint8_t*)buffer, size);
                if (return_value > 0) {
                    break;
                }
            }
        }
        return return_value;
    }
    size_t readBytes(uint8_t* buffer, size_t size) override
    {
        size_t return_value = 0;
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                return_value = serverClients[i].read(buffer, size);
                if (return_value > 0) {
                    break;
                }
            }
        }
        return return_value;
    }
    
    size_t write(uint8_t c) override
    {
        size_t last_return_value = 0;
        //push UART data to all connected telnet clients
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++) {
            if (serverClients[i] && serverClients[i].connected()){
                last_return_value = serverClients[i].write(c);
                feedLoopWDT(); // Must be done <= 360 mS
                auto _millis = millis();
                if(_millis > _last_yield_ms + _yield_timeout) {
                    //yield();
                    delay(10);
                    _last_yield_ms=_millis;
                }
            }
        }
        return last_return_value;
    }
    size_t write(const uint8_t *buffer, size_t size) override
    {
        size_t last_return_value = 0;
        //push UART data to all connected telnet clients
        for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
                last_return_value = serverClients[i].write(buffer, size);
                feedLoopWDT(); // Must be done <= 360 mS
                auto _millis = millis();
                if(_millis > _last_yield_ms + _yield_timeout) {
                    //yield();
                    delay(10);
                    _last_yield_ms=_millis;
                }
            }
        }
        return last_return_value;
    }
    using Print::write; // Import other write() methods to support things like write(0) properly

protected:
    WiFiServer server;
    WiFiClient serverClients[MAX_TELNET_CLIENTS];
    size_t _rx_size;
    unsigned long _last_yield_ms;
    const unsigned long _yield_timeout = 500;
};

class TelnetSerial: public Stream
{
public:
    TelnetSerial(uint16_t telnetPort = 23) : _telnet(telnetPort) {

    }
    virtual ~TelnetSerial() {}

    void begin() {
        _telnet.begin();
        _begun = true;
    }

    void end() {
        _telnet.end();
        _begun = false;
    }

    void loop() {
        SCOPED_STACK_ENTRY;
        _telnet.handleConnections();
    }

    int available(void)
    {
        int last_return_value = 0;
        last_return_value = Serial.available();
        if(_begun && !last_return_value) {
            last_return_value = _telnet.available();
        }
        return last_return_value;
    }

    void flush(void)
    {
        Serial.flush();
        if(_begun) {
            _telnet.flush();
        }
    }

    int peek(void) override
    {
        int last_return_value = -1;
        last_return_value = Serial.peek();
        if(_begun && last_return_value < 0) {
            last_return_value = _telnet.peek();
        }
        return last_return_value;
    }

    int read(void) override
    {
        int last_return_value = -1;
        last_return_value = Serial.read();
        if(_begun && last_return_value < 0) {
            last_return_value = _telnet.read();
        }
        return last_return_value;
    }
    
    size_t read(char* buffer, size_t size)
    {
        size_t last_return_value = 0;
        last_return_value = Serial.read(buffer, size);
        if(_begun && !last_return_value) {
            last_return_value = _telnet.read(buffer, size);
        }
        return last_return_value;
    }
    size_t readBytes(char* buffer, size_t size) override
    {
        size_t last_return_value = 0;
        last_return_value = Serial.read(buffer, size);
        if(_begun && !last_return_value) {
            last_return_value = _telnet.read(buffer, size);
        }
        return last_return_value;
    }
    
    size_t write(uint8_t c) override
    {
        size_t last_return_value = 0;
        //. Test if using single char writing...
        //if(_begun) {
        //    last_return_value = _telnet.write(c);
        //}
        last_return_value = Serial.write(c);
        return last_return_value;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        size_t last_return_value = 0;
        if(_begun) {
            last_return_value = _telnet.write(buffer, size);
        }
        last_return_value = Serial.write(buffer, size);
        return last_return_value;
    }

    using Print::write; // Import other write() methods to support things like write(0) properly

protected:
    Telnet _telnet;
    bool _begun = false;

public:
    Telnet& TelNet = _telnet;
};


#endif
