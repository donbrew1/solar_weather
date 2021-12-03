/**
 * httpUpdate.ino
 *
 *  Created on: 27.11.2015
 *
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

//#define VERSION "v1.3.2"
//#define HOST "solarweather"
const char* urlBase = "http://192.168.1.207:5000/update";

const char* ssid = "Dhome";
const char* password = "TiffanyZ7";
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;
WiFiClient  client;

uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
  static uint16_t attempt = 0;
  Serial.print("Connecting to ");
  if (nSSID) {
    WiFi.begin(nSSID, nPassword);
    Serial.println(nSSID);
  }

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 50)
  {
    delay(2000);
    Serial.print(".");
  }
  ++attempt;
  Serial.println("");
  if (i == 51) {
    Serial.print("Connection: TIMEOUT on attempt: ");
    Serial.println(attempt);
    if (attempt % 2 == 0)
      Serial.println("Check if access point available or SSID and Password\r\n");
    return false;
  }
  Serial.println("Connection: ESTABLISHED");
  Serial.print("Got IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void Awaits()
{
  uint32_t ts = millis();
  while (!connection_state)
  {
    delay(500);
    if (millis() > (ts + reconnect_interval) && !connection_state) {
      connection_state = WiFiConnect();
      ts = millis();
    }
  }
}
//in setup:
// uint32_t startTime = millis();
//  connection_state = WiFiConnect(ssid, password);
//  if (!connection_state) // if not connected to WIFI
//    Awaits();          // constantly trying to connect 



void checkForUpdates(void)
{
  String checkUrl = String( urlBase);
  checkUrl.concat( "?ver=" + String(VERSION) );
  checkUrl.concat( "&dev=" + String(HOST) );

  Serial.println("INFO: Checking for updates at URL: " + String( checkUrl ) );
  
  t_httpUpdate_return ret = ESPhttpUpdate.update( checkUrl );

  switch (ret) {
    default:
    case HTTP_UPDATE_FAILED:
      Serial.println("ERROR: HTTP_UPDATE_FAILD Error (" + String(ESPhttpUpdate.getLastError()) + "): " + ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("INFO: HTTP_UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("INFO status: HTTP_UPDATE_OK");
      break;
    }
}
