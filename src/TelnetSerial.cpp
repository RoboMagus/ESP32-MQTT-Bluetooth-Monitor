/*
 TelnetSerial.cpp - Arduino like serial library for printing over Telnet for ESP8266 & ESP32
 */

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <inttypes.h>
#include "Arduino.h"
#include "TelnetSerial.h"
#include "Esp.h"

Telnet::Telnet(uint16_t port) : server(port)
{

}

void Telnet::begin()
{
    end();

    // Start Ser2Net server
    server.setTimeout(15); // 15 seconds...
    server.begin();
    // Keep delay enabled to combine smaller packages and thus reduce WiFi traffic!
    // server.setNoDelay(true);   
}
    
void Telnet::end()
{
    for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++) {
        if(serverClients[i]) {
            serverClients[i].stop();
        }
    }
    server.stop();
}

void Telnet::handleConnections()
{
  // Check if there are any new clients ---------
  if (server.hasClient()){
    for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]){
          serverClients[i].stop();
        }
        serverClients[i] = server.available();
        continue;
      }
    }
    // No free/disconnected spot so reject
    server.available().stop();
  }
}

int Telnet::getConnectionCount()
{
  int cnt = 0;     
  for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
          cnt++;
      }
  }
  return cnt;
}

int Telnet::available(void)
{
    int return_value = 0;
    for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
            return_value = serverClients[i].available();
            if (return_value > 0) {
                break;
            }
        }
    }
    return return_value;
}

void Telnet::flush(void)
{
  for(uint8_t i = 0; i < MAX_TELNET_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      serverClients[i].flush();
    }
  }
}