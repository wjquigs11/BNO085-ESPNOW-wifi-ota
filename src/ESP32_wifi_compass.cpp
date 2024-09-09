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
#include <NTPClient.h>

#include "compass.h"

File consLog;
using namespace reactesp;
ReactESP app;
RepeatReaction* checkCompassReact;

float lastValue = 0;

#define VARIATION -15.3

Preferences preferences;

bool initWiFi();
void startAP();

#define HTTP_PORT 80
extern AsyncWebServer server;
extern bool serverStarted;
extern AsyncEventSource events;
void startWebServer();
extern String host;
extern JSONVar readings;
String getSensorReadings();

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
extern compass_s compassParams;

// timing
#define DEFDELAY 50 // for compass driver update
unsigned long WebTimerDelay = 100; // for display
unsigned long previousMillis, previousDisplay, previousReading;
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

#ifdef ESPNOW
void setupESPNOW(const char *ssid);
void loopESPNOW();
#endif

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


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
int totalReports;
int goodReports;

float getCompassHeading(int variation, int orientation) {
  float retcode = -9.9;
  if (!compassReady) {
    return -1.0;    
  }
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
    goodReports++; totalReports++;
    if (gameRot) {
      heading = calculateHeading(sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k, variation + orientation);
#ifdef DEBUG
      Serial.printf("%d game vector %.2f %.2f %.2f %.2f ", sensorValue.sensorId, sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
      //Serial.printf("heading: %.2f deg\n", heading);
#endif
      retcode = heading;
    }
    break;
  case SH2_ROTATION_VECTOR:
    goodReports++; totalReports++;
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
      retcode = heading;
    }
    break;
  default:
    totalReports++;
    logToAll("unknown sensor ID: %d" + String(sensorValue.sensorId));
    break;
  }
  // configure readings for web page server sent events (SSE)
  readings["bearing"] = String(heading,0);
  readings["variation"] = compassParams.variation;
  readings["orientation"] = compassParams.orientation;
  readings["frequency"] = compassParams.frequency;
  readings["calstatus"] = calStatus;
  //logToAll("readings: " + JSON.stringify(readings));
  if (gameRot && absRot)
    printf("difference %.2f\n", abs(heading - heading2));
  return retcode;
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
  compassReady = bno08x.begin_I2C(0x4A, &Wire, 0);
  if (!compassReady) {
    Serial.println("BNO08x not found");
    i2cScan(Wire);
  }
#endif
  if (compassReady) {
    logToAll("BNO08x Found\n");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
      logToAll(logString);
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

#ifdef ESPNOW
  setupESPNOW(WiFi.SSID().c_str());
#endif

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
  // reading ESPNOW peer MAC in espnow.cpp

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

  // Update the time
  while(!timeClient.update()) {
      timeClient.forceUpdate();
  }
  logToAll(timeClient.getFormattedDate());

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

  checkCompassReact = app.onRepeat(compassParams.frequency, []() {
    heading = getCompassHeading(compassParams.variation, compassParams.orientation);
    if (heading <-3) logToAll("heading error: " + String(heading));
#ifdef N2K
    SendN2kCompass(heading);
#endif
  });  

  app.onRepeat(100, []() {
    //PollCANStatus();
#ifdef N2K
    n2kesp->ParseMessages();
#endif
#ifdef ESPNOW
    loopESPNOW();
#endif
  });

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
