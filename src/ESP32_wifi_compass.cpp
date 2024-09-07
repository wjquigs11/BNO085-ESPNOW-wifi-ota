
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

File consLog;
using namespace reactesp;
ReactESP app;
RepeatReaction* n2kReact;

float lastValue = 0;

#define VARIATION -15.3

Preferences preferences;

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted = false;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)
AsyncEventSource events("/events");
extern void startWebServer();
String host = "mastcomp";
JSONVar readings;
String getSensorReadings();

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

bool gameRot, absRot;

float calculateHeading(float r, float i, float j, float k, int correction);
float calculateHeading2(float r, float i, float j, float k, int correction);
float heading, heading2, accuracy;
int calStatus;

#ifdef SH_ESP32
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
TwoWire *i2c;
// for SPI BNO compass
#define RESET 27 // pull low to reset
#define SDA 21
#define SCL 22
// mode pins pull high for SPI
//#define SPI0 21
//#define SPI1 22
//#define INT 14
//static const uint8_t SS    = 5;
//static const uint8_t MOSI  = 23;
//static const uint8_t MISO  = 19;
//static const uint8_t SCK   = 18;
// for display
#define SDA_PIN 16
#define SCL_PIN 17
Adafruit_SSD1306 *display;
#else
#define RESET -1
//SDA 21
//SCL 22
#endif
Adafruit_BNO08x bno08x(RESET);
sh2_SensorValue_t sensorValue;
bool compassReady = false;
control_s compassParams;

// timing
#define DEFDELAY 50 // for compass driver update
unsigned long WebTimerDelay = 100; // for display
unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
//int minReadRate = BNOREADRATE;
int minReadRate = DEFDELAY;
unsigned long currentMillis = millis();
unsigned long start;                // used to measure ESPNOW pairing time

#ifdef N2K
String can_state;
int num_n2k_sent = 0, num_n2k_messages = 0;
void SendN2kCompass(float heading);
void setupN2K();
extern tNMEA2000 *n2kesp;
#endif

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

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void logToAll(String s) {
  if (s.endsWith("\n")) {
    Serial.print(s);
    consLog.print(s);
    if (serverStarted)
      WebSerial.print(s);
  } else {
    Serial.println(s);
    consLog.println(s);
    if (serverStarted)
      WebSerial.println(s);
  }
}

#ifdef N2K
void PollCANStatus() {
  // CAN controller registers are SJA1000 compatible.
  // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUN";
      break;
    case 1:
      can_state = "OFF";
      // try to automatically recover
      //RecoverFromCANBusOff();
      break;
  }
}
#endif

// BNO: define the sensor outputs we want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (gameRot)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
      Serial.println("Could not enable game vector");
    else
      Serial.println("enabled game vector");
  if (absRot)
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100))
      Serial.println("Could not enable rotation vector");
    else
      Serial.println("enabled abs rotation vector");
}

// get heading from the compass
// return values:
// 0.0 to 360.0 = heading
// -1.0 = sensor not found
// -2.0 = minimum delay in case displayDelay is set too low
// -3.0 = error reading sensor
// -4.0 = unexpected report from sensor
float getCompassHeading(int variation, int orientation) {
  if (!compassReady)
    return -1.0;
  unsigned long currentMillis = millis();
  unsigned long deltaM = currentMillis - previousReading;
  if (deltaM < BNOREADRATE) {
    logToAll("reading too soon: " + String(currentMillis) + "-" + String(previousReading) + "=" + String(deltaM) + "\n");
    return -2.0; // minimum delay in case displayDelay is set too low
  } //else logToAll("getCompassHeading");
  previousReading = currentMillis;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  /* Status of a sensor
   *   0 - Unreliable
   *   1 - Accuracy low
   *   2 - Accuracy medium
   *   3 - Accuracy high
   */
  switch (sensorValue.sensorId) {
  case SH2_GAME_ROTATION_VECTOR:
    if (gameRot) {
      heading = calculateHeading(sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k, variation + orientation);
#ifdef DEBUG
      printf("game vector %.2f %.2f %.2f %.2f ", sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
      printf("heading: %.2f deg\n", heading);
#endif
      return heading;
    }
    break;
  case SH2_ROTATION_VECTOR:
    calStatus = sensorValue.status;
    if (absRot) {
      accuracy = sensorValue.un.rotationVector.accuracy;
      heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, variation + orientation);
      //Serial.printf("abs vector status %d %.2f %.2f %.2f %.2f\n", calStatus, sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
      //logToAll("heading1: " + String(heading) + "  cal: " + calStatus + "\n");
#ifdef DEBUG
      heading2 = calculateHeading2(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);
      printf("rota vector status %d %.2f %.2f %.2f %.2f ", calStatus, sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
      printf("heading2: %.2f degrees, accuracy %.2f/%.2f r/d, status %d\n", heading2, accuracy, accuracy * 180.0 / M_PI, calStatus);
#endif
      // configure readings for web page server sent events (SSE)
      readings["bearing"] = String(heading,0);
      readings["variation"] = compassParams.variation;
      readings["orientation"] = compassParams.orientation;
      readings["frequency"] = compassParams.frequency;
      readings["calstatus"] = calStatus;
      //logToAll("readings: " + JSON.stringify(readings));
      return heading;
    }
    break;
  default:
    logToAll("unknown sensor ID: %d" + String(sensorValue.sensorId));
    break;
  }
  if (gameRot && absRot)
    printf("difference %.2f\n", abs(heading - heading2));
  return -4.0;
}

// Function to calculate tilt-compensated heading from a quaternion
float calculateHeading(float r, float i, float j, float k, int correction) {
  // Convert quaternion to rotation matrix
  float r11 = 1 - 2 * (j * j + k * k);  
  float r21 = 2 * (i * j + r * k);
  float r31 = 2 * (i * k - r * j);
  float r32 = 2 * (j * k + r * i);
  float r33 = 1 - 2 * (i * i + j * j);

  // Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);
  // Calculate yaw (psi)
  float psi = atan2(-r21, r11);

  double heading = (psi * 180 / M_PI) + correction;
  // correction may be positive or negative
  if (heading > 359) heading -= 360;
  if (heading < 0) heading += 360;
  return (double)heading;
}

float calculateHeading2(float r, float i, float j, float k, int correction) {
  float heading = atan2(-2.0 * (i * j + k * r), r * r - i * i - j * j + k * k); // in radians
  heading *= (180.0 / M_PI);                                                   // convert to degrees
  heading += correction;
  if (heading < 0) heading += 360.0;
  return heading;
}

void i2cScan(TwoWire Wire);
void WebSerialonMessage(uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("I2C compass");
#ifdef SH_ESP32 // use display
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else Serial.println(F("SSD1306 OK"));
  delay(100);
  display->setRotation(3);
  display->clearDisplay();
  display->printf("SH-ESP32\nN2K\nCOMPASS\n");
  display->display();
#endif
if (RESET>0) { // init compass if we have a RESET pin
  pinMode(RESET, OUTPUT);
  // pull RST low to reset BNO
  digitalWrite(RESET, LOW);
  delay(50);
  digitalWrite(RESET, HIGH);
  delay(100); // let BNO digest reset
}
#ifdef SPICOMPASS
  Serial.println("SPI compass");
  compassReady = bno08x.begin_SPI(SS, INT);
#endif  
  // 0x4A for Adafruit
  // 0x4B for Teyleten
#ifdef SH_ESP32
  Wire1.begin(SDA,SCL);
  if (!(compassReady = bno08x.begin_I2C(0x4B, &Wire1, 0)))
    i2cScan(Wire1);
#else
  if (!(compassReady = bno08x.begin_I2C(0x4A, &Wire, 0)))
    i2cScan(Wire);
#endif
  if (compassReady) {
    logToAll("BNO08x Found\n");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
      logToAll(logString + "\n");
    }
    setReports();
  }
  if (SPIFFS.begin())
    Serial.println("opened SPIFFS");
  else
    Serial.println("failed to open SPIFFS");

#ifdef N2K  // moving here to give drivers more time to initialize
  setupN2K();
#endif

  if (initWiFi()) {
    startWebServer();
  } else {
    startAP();
  }

  // start a console.log file in case we crash before Webserial starts
  if (SPIFFS.exists("/console.log")) {
    SPIFFS.remove("/console.log");
  }
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog) {
    Serial.println("failed to open console log");
  }
  if (consLog.println("ESP compass console log.")) {
    Serial.println("console log written");
  } else {
    Serial.println("console log write failed");
  }
  logToAll("BNO08x based tilt-compensated compass\n");

  preferences.begin("ESPcompass", false);
  compassParams.frequency = preferences.getInt("frequency", DEFDELAY);
  if (compassParams.frequency < minReadRate) compassParams.frequency = minReadRate;
  if (compassParams.frequency > 1000) compassParams.frequency = 1000;
  logToAll("frequency " + String(compassParams.frequency) + "\n");
  //Serial.printf("frequency %d\n", compassParams.frequency);
  compassParams.variation = preferences.getInt("variation", VARIATION);
  logToAll("variation " + String(compassParams.variation) + "\n");
  //Serial.printf("variation %d\n", compassParams.variation);
  compassParams.orientation = preferences.getInt("orientation", 0);
  logToAll("orientation " + String(compassParams.orientation) + "\n");
  //Serial.printf("orientation %d\n", compassParams.orientation);
  WebTimerDelay = preferences.getInt("timerdelay", 1000);
  if (WebTimerDelay<200) {
    WebTimerDelay = 200;
    preferences.putInt("timerdelay", 200);
  }
  logToAll("WebTimerDelay " + String(WebTimerDelay) + "\n");
  //Serial.printf("WebTimerDelay %d\n", WebTimerDelay);
  gameRot = preferences.getBool("gameRot", false);
  logToAll("gameRot " + String(gameRot) + "\n");
  //Serial.printf("gameRot %d\n", gameRot);
  absRot = preferences.getBool("absRot", true);
  logToAll("absRot " + String(absRot) + "\n");
  //Serial.printf("absRot %d\n", absRot);
  host = preferences.getString("hostname", host);
  logToAll("hostname: " + host + "\n");
  
  if (!MDNS.begin(host.c_str()))
    logToAll(F("Error starting MDNS responder\n"));
  else {
    logToAll("MDNS started " + host + "\n");
  }

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT))
    logToAll("MDNS add service failed\n");

  int n = MDNS.queryService("http", "tcp");
  if (n == 0) {
    logToAll("No services found");
  } else {
    for (int i = 0; i < n; i++) {
      logToAll("Service found: ");
      logToAll(MDNS.hostname(i) + " (" + String(MDNS.IP(i)) + ":" + String(MDNS.port(i)) + ")\n");
    }
  }

  ElegantOTA.begin(&server);

  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello from your compass!"); });

  // Initialize WebSerial
  WebSerial.begin(&server);
  // Attach a callback function to handle incoming messages
  WebSerial.onMessage(WebSerialonMessage);

  server.begin();
  serverStarted = true;
  logToAll("HTTP server started @" + WiFi.localIP().toString() + "\n");

  consLog.flush();

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    if (compassReady) {
      // Send Events to the client with the Sensor Readings Every x msecs
      //events.send("ping",NULL,millis());
      events.send(getSensorReadings().c_str(),"new_readings" ,millis());
      //WebSerial.printf("heading: %.2f d, %0.2f r, accuracy %.2f/%.2f r/d, cal status %d\n", heading, heading*DEGTORAD, accuracy, accuracy*180.0/M_PI, calStatus);
    }
    consLog.flush();
});

#ifdef N2K
  Serial.printf("starting N2K xmit reaction\n");
#endif
  n2kReact = app.onRepeat(compassParams.frequency, []() {
    heading = getCompassHeading(compassParams.variation, compassParams.orientation);
#ifdef N2K
    SendN2kCompass(heading);
#endif
  });  

#ifdef N2K
  Serial.printf("starting N2K parse reaction\n");
  app.onRepeat(1, []() {
    //PollCANStatus();
    n2kesp->ParseMessages();
  });
#endif

  app.onRepeat(100, []() {
    ElegantOTA.loop();  
    WebSerial.loop();
  });


#ifdef SH_ESP32
  // update results
  app.onRepeat(1000, []() {
    display->clearDisplay();
    display->setTextSize(1);
    display->setCursor(0, 0);
    display->setTextColor(SSD1306_WHITE);
#ifdef N2K
    display->printf("CAN: %s\n", can_state.substring(0, 3).c_str());
    display->printf("TX: %d\n", num_n2k_sent);
    display->printf("RX: %d\n", num_n2k_messages);
#endif
    display->printf("Up: %lu\n", millis() / 1000);
    display->printf("\n");
    display->setTextSize(2);
    display->printf("%.1f\n", heading);
    display->setTextSize(1);
    display->printf("%.1f\n", accuracy*180.0/M_PI);
    display->printf("%d\n", calStatus);
    display->display();
#ifdef N2K
    num_n2k_sent = num_n2k_messages = 0;
#endif
  });
#endif

} // setup

// with ReactESP everything is scheduled through app lambdas
void loop() { app.tick(); }
