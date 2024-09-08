#include <Arduino.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <esp_wifi.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ReactESP.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "elapsedMillis.h"
#include <NMEA2000_esp32.h>

#include "compass.h"

extern unsigned long previousMillis;


// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";

#define MAX_NETS 2
int num_nets;
String ssid[MAX_NETS] = {"null8chars", "null8chars"};
String pass[MAX_NETS];
String ip[MAX_NETS];
String gateway[MAX_NETS];

String connectSSID; // where we got a connection

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted = false;
AsyncEventSource events("/events");
void startWebServer();
String host = "mastcomp";
JSONVar readings;
String getSensorReadings();

void logToAll(String S);

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
}

bool readWiFi() {
  File file = SPIFFS.open(ssidPath);
  if(!file || file.isDirectory()) {
    logToAll("failed to open ssid for reading");
    return false;
  }
  int i=0;
  while(file.available()) {
    ssid[i] = file.readStringUntil('\n');
    logToAll("found SSID " + ssid[i]);
    i++;
  }
  num_nets=i;
  i=0;
  file = SPIFFS.open(passPath);
  if(!file || file.isDirectory()) {
    logToAll("failed to open pass for reading");
    return false;
  }
  i=0;
  while(file.available()) {
    pass[i] = file.readStringUntil('\n');
    logToAll("found passwd " + pass[i]);
    i++;
  }
  if (i != num_nets) {
    logToAll("Number of SSIDs and passwords do not match");
    return false;
  }
  i=0;
  file = SPIFFS.open(ipPath);
  if(!file || file.isDirectory()) {
    logToAll("failed to open IP for reading (ok)");
  }
  i=0;
  while(file.available()) {
    ip[i++] = file.readStringUntil('\n');
  }
  i=0;
  file = SPIFFS.open(gatewayPath);
  if(!file || file.isDirectory()) {
    logToAll("failed to open gateway for reading (ok)");
  }
  i=0;
  while(file.available()) {
    gateway[i++] = file.readStringUntil('\n');
  }
  logToAll("found " + String(num_nets) + " networks");
  for (i=0; i<num_nets; i++) {
    logToAll("wifi[" + String(i) + "]: " + ssid[i] + " " + pass[i] + " " + ip[i] + " " + gateway[i]);
  }
  //logToAll("return readwifi");
  return true;
}

bool initWiFi() {
  int num_tries = 0;
  const int MAX_TRIES = 5;

  if (!readWiFi()) {
    Serial.println("Failed to read WiFi credentials");
    return false;
  }
  WiFi.mode(WIFI_STA);
  for (int i=0; i<num_nets; i++) {
    Serial.printf("Found SSID %d: %s\n", i, ssid[i].c_str());
    if(ssid[i]=="") {
      Serial.println("Undefined SSID.");
      return false;
    }
    localIP.fromString(ip[i].c_str());
    localGateway.fromString(gateway[i].c_str());
    if (!WiFi.config(localIP, localGateway, subnet)) {
      Serial.println("STA Failed to configure");
      return false;
    }
    Serial.printf("Connecting to WiFi %s with pass %s\n", ssid[i].c_str(), pass[i].c_str());
    WiFi.begin(ssid[i].c_str(), pass[i].c_str());

    unsigned long currentMillis = millis();
    previousMillis = currentMillis;

    while(WiFi.status() != WL_CONNECTED && num_tries++ < MAX_TRIES) {
      delay(1000);
      Serial.print(".");
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        Serial.println("Failed to connect.");
        //return false;
      }
    }
    num_tries = 0;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(" connected: ");
      Serial.println(WiFi.localIP());
      return true;
    }
  }
  return false;
}

void startAP() {
    // Connect to Wi-Fi network with SSID and password
    logToAll("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    logToAll("AP IP address: " + IP.toString());

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
  
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
#if 0
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
#endif
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router");
      delay(3000);
      ESP.restart();
    });
    server.begin();
}
