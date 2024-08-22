/*

*/
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiMulti.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <math.h>
#include "Async_ConfigOnDoubleReset_Multi.h"

double rad=0.01745;

extern Preferences preferences;     

extern AsyncWebServer server;
extern bool serverStarted;
extern AsyncEventSource events;
extern JSONVar readings;
extern String host;

// Timer variables
#define DEFDELAY 1000
extern unsigned long lastTime;
extern int WebTimerDelay;
extern int minReadRate;

// Get Sensor Readings and return JSON object
String getSensorReadings() {
  // readings set in getMastHeading()
  //readings["sensor"] = "sensor";
  String jsonString = JSON.stringify(readings);
  //Serial.println(readings);
  return jsonString;
}

// Replaces placeholder with DHT values
String processor(const String& var){
  //Serial.println(var);
  if(var == "VARIATION"){
    return String(0);
  }
  else if(var == "ORIENTATION"){
    return String(1);
  }
  else if(var == "TIMERDELAY"){
    return String(WebTimerDelay);
  }
  return String();
}

void logToAll(String S);

void startWebServer() {
  Serial.println("starting web server");

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  // Request latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSensorReadings();
    //logToAll("getSensorReadings\n");
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host;
    buf += " ESP local MAC addr: " + String(WiFi.macAddress() + "\n");
    logToAll(buf);
    request->send_P(200, "text/plain", buf.c_str());
    buf = String();
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    logToAll("config\n");
    String response = "none";
    if (request->hasParam("hostname")) {
      Serial.printf("hostname %s\n", request->getParam("hostname")->value().c_str());
      host = request->getParam("hostname")->value();
      response = "change hostname to " + host + "\n";
      logToAll(response);
      preferences.putString("hostname",host);
    }
    request->send_P(200, "text/plain", response.c_str());
    response = String();
  }); 

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);
}