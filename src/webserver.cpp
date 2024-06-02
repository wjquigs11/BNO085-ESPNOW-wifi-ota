
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
//#include <Adafruit_BNO08x.h>
#include <math.h>
#include "Async_ConfigOnDoubleReset_Multi.h"
//#include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager

double rad=0.01745;

#define VARIATION -15.2
static int variation;
static int orientation; // how is the compass oriented on the board

extern Preferences preferences;     

extern AsyncWebServer server;
extern AsyncEventSource events;
extern JSONVar readings;
extern String host;

// Timer variables
#define DEFDELAY 1000
extern unsigned long lastTime;
extern int timerDelay;

extern bool gameRot, absRot;

float calculateHeading(float r, float i, float j, float k, int correction);
float calculateHeading2(float r, float i, float j, float k);
void setReports();
extern float heading, heading2, accuracy;

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
    return String(variation);
  }
  else if(var == "ORIENTATION"){
    return String(orientation);
  }
  else if(var == "TIMERDELAY"){
    return String(timerDelay);
  }
  return String();
}

void startWebServer() {
  Serial.println("starting web server");

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSensorReadings();
    Serial.println("getSensorReadings");
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host + ", variation: " + String(variation) + ", orientation: " + String(orientation) + ", timerdelay: " + String(timerDelay);
    Serial.print("hostname: ");
    Serial.println(host.c_str());
    request->send_P(200, "text/plain", buf.c_str());
  });
// switching config to GET because it's easier
//  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("orientation")) {
      orientation = atoi(request->getParam("orientation")->value().c_str());
      Serial.printf("change orientation to %d\n", orientation);
      preferences.putInt("orientation", orientation);
    } else if (request->hasParam("variation")) {
      variation = atoi(request->getParam("variation")->value().c_str());
      Serial.printf("change variation to %d\n", variation);
      preferences.putInt("variation",variation);
    } else if (request->hasParam("timerdelay")) {
      timerDelay = atoi(request->getParam("timerdelay")->value().c_str());
      Serial.printf("change timerdelay to %d\n", timerDelay);
      preferences.putInt("timerdelay",timerDelay);
    } else if (request->hasParam("gameRot")) {
      gameRot = (request->getParam("gameRot")->value().equals("true")) ? true : false;
      Serial.printf("change game to %d\n", gameRot);
      preferences.putBool("gameRot",gameRot);
      setReports();
    } else if (request->hasParam("absRot")) {
      Serial.printf("absrot %s\n", request->getParam("absRot")->value().c_str());
      absRot = (request->getParam("absRot")->value().equals("true")) ? true : false;
      Serial.printf("change abs to %d\n", absRot);
      preferences.putBool("absRot",absRot);
      setReports();
    } 
    request->send(SPIFFS, "/index.html", "text/html");
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