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
#include "compass.h"
//#include <esp_now.h>
#include <ReactESP.h>

double rad=0.01745;

extern Preferences preferences;     

extern AsyncWebServer server;
extern bool serverStarted;
extern AsyncEventSource events;
extern JSONVar readings;
extern String host;
using namespace reactesp;
extern ReactESP app;
extern RepeatReaction* n2kReact;

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

void logToAll(String S);
float getCompassHeading(int variation, int orientation);
void SendN2kCompass(float heading);

// Get Sensor Readings and return JSON object
String getSensorReadings() {
  // readings set in getMastHeading()
  String jsonString = JSON.stringify(readings);
  //logToAll("gSreadings: " + jsonString);
  return jsonString;
}

String processor(const String& var) {
  Serial.println(var);
  if(var == "VARIATION") {
    return String(compassParams.variation);
  }
  else if(var == "ORIENTATION") {
    return String(compassParams.orientation);
  }
  else if(var == "TIMERDELAY") {
    return String(WebTimerDelay);
  }
  else if(var == "FREQUENCY") {
    return String(compassParams.frequency);
  }
  return String();
}

void startWebServer() {
  logToAll("starting web server");

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    logToAll("index.html");
    request->send(SPIFFS, "/index.html", "text/html", false, processor);
  });

  // Request latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSensorReadings();
    //logToAll("getSensorReadings\n");
    //logToAll(readings);
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/compass", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host + ", variation: " + String(compassParams.variation) + ", orientation: " + String(compassParams.orientation) + ", timerdelay: " + String(WebTimerDelay) + ", frequency: " + String(compassParams.frequency) + "\n";
    buf += "ESP local MAC addr: " + String(WiFi.macAddress() + "\n");
    logToAll(buf);
    request->send(200, "text/plain", buf.c_str());
    buf = String();
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    logToAll("config\n");
    String response = "none";
    if (request->hasParam("orientation")) {
      compassParams.orientation = atoi(request->getParam("orientation")->value().c_str());
      response = "change orientation to " + String(compassParams.orientation) + "\n";
      logToAll(response);
      preferences.putInt("orientation", compassParams.orientation);
    } else if (request->hasParam("variation")) {
      compassParams.variation = atoi(request->getParam("variation")->value().c_str());
      response = "change variation to " + String(compassParams.variation) + "\n";
      logToAll(response);
      preferences.putInt("variation",compassParams.variation);
    } else if (request->hasParam("timerdelay")) {
      WebTimerDelay = atoi(request->getParam("timerdelay")->value().c_str());
      if (WebTimerDelay < minReadRate) WebTimerDelay = minReadRate;
      response = "change WebTimerDelay to " + String(WebTimerDelay) + "\n";
      logToAll(response);
      preferences.putInt("timerdelay",WebTimerDelay);
    } else if (request->hasParam("gameRot")) {
      gameRot = (request->getParam("gameRot")->value().equals("true")) ? true : false;
      response = "change gameRot to " + String(gameRot) + "\n";
      logToAll(response);
      preferences.putBool("gameRot",gameRot);
      setReports();
    } else if (request->hasParam("absRot")) {
      Serial.printf("absrot %s\n", request->getParam("absRot")->value().c_str());
      absRot = (request->getParam("absRot")->value().equals("true")) ? true : false;
      response = "change absRot to " + String(absRot) + "\n";
      logToAll(response);
      preferences.putBool("absRot",absRot);
      setReports();
    }  else if (request->hasParam("hostname")) {
      Serial.printf("hostname %s\n", request->getParam("hostname")->value().c_str());
      host = request->getParam("hostname")->value();
      response = "change hostname to " + host + "\n";
      logToAll(response);
      preferences.putString("hostname",host);
      logToAll("preferences " + preferences.getString("hostname", "unknown") + "\n");
    }  else if (request->hasParam("frequency")) {
      compassParams.frequency = atoi(request->getParam("frequency")->value().c_str());
      if (compassParams.frequency < BNOREADRATE) compassParams.frequency = BNOREADRATE;
      Serial.printf("frequency %d\n", compassParams.frequency);
      response = "change frequency to " + String(compassParams.frequency) + "\n";
      logToAll(response);
      preferences.putInt("frequency",compassParams.frequency); 
      logToAll("restarting N2K reaction");
      // now delete/create reaction 
      app.remove(n2kReact);
      n2kReact = app.onRepeat(compassParams.frequency, []() {
        heading = getCompassHeading(compassParams.variation, compassParams.orientation);
#ifdef N2K
        SendN2kCompass(heading);
#endif
      });
    }  else if (request->hasParam("toggle")) {
      Serial.printf("toggle %s\n", request->getParam("toggle")->value().c_str());
      compassParams.compassOnToggle = (request->getParam("toggle")->value().equals("true")) ? true : false;
      response = "change toggle to " + String(compassParams.compassOnToggle) + "\n";
      logToAll(response);
      preferences.putBool("toggle",compassParams.compassOnToggle);
    } 
    //request->send(SPIFFS, "/index.html", "text/html");
    //request->redirect("/index.html");
    request->send(200, "text/plain", response.c_str());
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


