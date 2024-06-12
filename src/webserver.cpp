
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

#define VARIATION -15.2
static int variation;
static int orientation; // how is the compass oriented on the board

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
    logToAll("getSensorReadings\n");
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host + ", variation: " + String(variation) + ", orientation: " + String(orientation) + ", timerdelay: " + String(WebTimerDelay) + "\n";
    logToAll(buf);
    request->send_P(200, "text/plain", buf.c_str());
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    logToAll("config\n");
    if (request->hasParam("orientation")) {
      orientation = atoi(request->getParam("orientation")->value().c_str());
      logToAll("change orientation to " + String(orientation) + "\n");
      preferences.putInt("orientation", orientation);
    } else if (request->hasParam("variation")) {
      variation = atoi(request->getParam("variation")->value().c_str());
      logToAll("change variation to " + String(variation) + "\n");
      preferences.putInt("variation",variation);
    } else if (request->hasParam("timerdelay")) {
      WebTimerDelay = atoi(request->getParam("timerdelay")->value().c_str());
      if (WebTimerDelay < minReadRate) WebTimerDelay = minReadRate;
      logToAll("change WebTimerDelay to " + String(WebTimerDelay) + "\n");
      preferences.putInt("timerdelay",WebTimerDelay);
    } else if (request->hasParam("gameRot")) {
      gameRot = (request->getParam("gameRot")->value().equals("true")) ? true : false;
      logToAll("change game to " + String(gameRot) + "\n");
      preferences.putBool("gameRot",gameRot);
      setReports();
    } else if (request->hasParam("absRot")) {
      Serial.printf("absrot %s\n", request->getParam("absRot")->value().c_str());
      absRot = (request->getParam("absRot")->value().equals("true")) ? true : false;
      logToAll("change abs to " + String(absRot) + "\n");
      preferences.putBool("absRot",absRot);
      setReports();
    }  else if (request->hasParam("hostname")) {
      Serial.printf("hostname %s\n", request->getParam("hostname")->value().c_str());
      host = request->getParam("hostname")->value();
      logToAll("change hostname to " + host + "\n");
      preferences.putString("hostname",host);
    } 
    //request->send(SPIFFS, "/index.html", "text/html");
    request->redirect("/index.html");
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