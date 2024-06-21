
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
#include <esp_now.h>
#include <esp_wifi.h>
#include "Async_ConfigOnDoubleReset_Multi.h"
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

float lastValue = 0;

#define VARIATION -15.3

Preferences preferences;

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted = false;
AsyncEventSource events("/events");
extern void setupWifi();
extern void startWebServer();
String host = "ESPcompass";
JSONVar readings;
extern void check_status();
String getSensorReadings();
extern DoubleResetDetector *drd;

bool gameRot, absRot;

float calculateHeading(float r, float i, float j, float k, int correction);
float calculateHeading2(float r, float i, float j, float k, int correction);
float heading, heading2, accuracy;
int calStatus;

#define SH_ESP32 // the only version with a display
#ifdef SH_ESP32
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
TwoWire *i2c;
// for SPI BNO compass
#define RESET 14 // pull low to reset
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

// ESPNOW
// MAC Address of the controller (with boat compass)
//uint8_t serverAddress[] = {0x08, 0xB6, 0x1F, 0xB8, 0xC4, 0xAC};
//uint8_t serverAddress[ESP_NOW_ETH_ALEN];
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t serverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // starts as broadcast, to be replaced by actual server
esp_now_peer_info_t peerInfo;
#define MAX_CHANNEL 11  // 11 in North America or 13 in Europe
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
int32_t channel;
//bool compassPeered = false;
enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;
control_s compassParams;
esp_now_peer_info_t peer;

// timing
#define DEFDELAY 50 // for compass driver update
unsigned long WebTimerDelay = 1000; // for display
unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
#define BNOREADRATE 20 // msecs for 50Hz rate; optimum for BNO08x calibration
int minReadRate = BNOREADRATE;
unsigned long currentMillis = millis();
unsigned long start;                // used to measure ESPNOW pairing time

#define N2K // if N2K defined, init CAN bus and send N2K Heading PGN
String can_state;
int num_n2k_messages = 0;

//int32_t getWiFiChannel(const char *ssid);

void SendN2kCompass(float heading);
void setupN2K();

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void logToAll(String s) {
  Serial.print(s);
  consLog.print(s);
  if (serverStarted)
    WebSerial.print(s);
}

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
  }
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
  calStatus = sensorValue.status;
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
    if (absRot) {
      accuracy = sensorValue.un.rotationVector.accuracy;
      heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, variation + orientation);
      //logToAll("heading1: " + String(heading) + "  cal: " + calStatus + "\n");
#ifdef DEBUG
      heading2 = calculateHeading2(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);
      printf("rota vector status %d %.2f %.2f %.2f %.2f ", calStatus, sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
      printf("heading2: %.2f degrees, accuracy %.2f/%.2f r/d, status %d\n", heading2, accuracy, accuracy * 180.0 / M_PI, calStatus);
#endif
      // configure readings for web page server sent events (SSE)
      readings["bearing"] = heading;
      readings["calstatus"] = calStatus;
      return heading;
    }
    break;
  default:
    printf("unknown sensor ID: %d\n", sensorValue.sensorId);
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
  float psi = atan2(r21, r11);

  double heading = (psi * 180 / M_PI) + correction;
  // correction may be positive or negative
  if (heading > 360) heading -= 360;
  if (heading < 0) heading += 360;
  return heading;
}

float calculateHeading2(float r, float i, float j, float k, int correction) {
  float heading = atan2(2.0 * (i * j + k * r), r * r - i * i - j * j + k * k); // in radians
  heading *= (180.0 / M_PI);                                                   // convert to degrees
  heading += correction;
  if (heading < 0) heading += 360.0;
  return heading;
}
void addPeer(const uint8_t * mac_addr, uint8_t chan){
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } else Serial.println("added peer");
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}
/*
void readGetMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
  for (int i=0; i<6; i++)
    clientAddress[i] = baseMac[i];
}
*/
// callback when ESPNOW data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  logToAll("\r\nLast Packet Send Status:\t");
  logToAll(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success: " : "Delivery Fail: ");
  logToAll(String(status) + "\n");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  logToAll("Packet received " + String(len) + " bytes: ");
  printMAC(mac_addr);
  memcpy(&compassParams, incomingData, sizeof(compassParams));
  logToAll("\ntoggle " + String(compassParams.compassOnToggle) + " orientation " + String(compassParams.orientation) + " frequency " + String(compassParams.frequency) + "\n");
  if (compassParams.frequency < minReadRate) compassParams.frequency = minReadRate;
  if (compassParams.frequency > 1000) compassParams.frequency = 1000;
  // storing here redundantly since it comes from the main controller
  preferences.putInt("orientation", compassParams.orientation);
  preferences.putInt("frequency", compassParams.frequency);
  const char *reply = host.c_str();
  Serial.printf("reply len %d %d\n", sizeof(reply), host.length());
  esp_err_t result = esp_now_send(mac_addr, (uint8_t *) reply, host.length());
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
  if (pairingStatus == PAIR_REQUEST) {  
    // we got a packet from server on this channel, save channel and add peer
    Serial.print("Pairing done channel");
    Serial.print(channel);    // channel used by the server
    Serial.print(" in ");
    Serial.print(millis()-start);
    Serial.println("ms");
    addPeer(mac_addr, channel); // add the server  to the peer list 
    #ifdef SAVE_CHANNEL
      lastChannel = pairingData.channel;
      EEPROM.write(0, pairingData.channel);
      EEPROM.commit();
    #endif  
    pairingStatus = PAIR_PAIRED; 
  }
}

// we call this repeatedly until we guess the correct channel
PairingStatus autoPairing() {
  switch(pairingStatus) {
    case PAIR_REQUEST: {
      Serial.print("Pairing request on channel "  );
      Serial.println(channel);

      // set WiFi channel   
      ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
      if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
      }

      // set callback routines
      esp_now_register_send_cb(OnDataSent);
      esp_now_register_recv_cb(OnDataRecv);
    
      addPeer(serverAddress, channel);
      // send hostname and hope we get a reply on this channel
      const char *reply = host.c_str();
      Serial.printf("broadcast sending len %d %d\n", sizeof(reply), host.length());
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) reply, host.length());
      Serial.printf("esp_now_send returns 0x%X\n", result);
      Serial.printf("%s\n", esp_err_to_name(result));
      previousMillis = millis();
      pairingStatus = PAIR_REQUESTED;
      break;
    }
    case PAIR_REQUESTED: {
      // time out to allow receiving response from server
      currentMillis = millis();
      if(currentMillis - previousMillis > 1000) {
        previousMillis = currentMillis;
        // time out expired,  try next channel
        channel++;
        if (channel > MAX_CHANNEL) channel = 1;
        pairingStatus = PAIR_REQUEST;
      }
    break;
    }
    case PAIR_PAIRED: {
      // nothing to do here 
    break;
    }
  }
  return pairingStatus;
}  

void i2cScan();
void WebSerialonMessage(uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("I2C compass");
#ifdef ESP32
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(1);
  display->clearDisplay();
  display->printf("SH-ESP32\nN2K\nCOMPASS\n");
  display->display();
#endif
  // init compass
  pinMode(RESET, OUTPUT);
  // pull RST low to reset BNO
  digitalWrite(RESET, LOW);
  delay(50);
  digitalWrite(RESET, HIGH);
  delay(100); // let BNO digest reset
#ifdef SPICOMPASS
  Serial.println("SPI compass");
  compassReady = bno08x.begin_SPI(SS, INT);
#endif  
  // 0x4A for Adafruit
  // 0x4B for Teyleten
  // compassReady = bno08x.begin_I2C(0x4A, i2c, 0);
  Wire1.begin(21,22);
  compassReady = bno08x.begin_I2C(0x4B, &Wire1, 0);
  if (!compassReady) {
    logToAll("Failed to find BNO08x.\n");
    i2cScan();
    //return;
  } else {
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

  // start a console.log file in case we crash before Webserial starts
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog)
    Serial.println("failed to open console log");
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
  compassParams.variation = preferences.getInt("variation", VARIATION);
  compassParams.orientation = preferences.getInt("orientation", 0);
  WebTimerDelay = preferences.getInt("timerdelay", 1000);
  if (WebTimerDelay<200) {
    WebTimerDelay = 200;
    preferences.putInt("timerdelay", 200);
  }
  gameRot = preferences.getBool("gameRot", false);
  absRot = preferences.getBool("absRot", true);
  host = preferences.getString("host", "ESPcompass");
  logToAll("hostname: " + host + "\n");

  setupWifi();
  if (!MDNS.begin(host.c_str()))
    logToAll(F("Error starting MDNS responder\n"));
  else {
    logToAll("MDNS started " + host + "\n");
  }
  // Serial.printf("MDNS started %s\n", host.c_str());

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT))
    logToAll("MDNS add service failed\n");

  startWebServer();

  ElegantOTA.begin(&server);

  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello from your compass!"); });

  // Initialize WebSerial
  WebSerial.begin(&server);
  // Attach a callback function to handle incoming messages
  WebSerial.onMessage(WebSerialonMessage);

  server.begin();
  serverStarted = true;
  logToAll("HTTP server started @" + String(WiFi.localIP()) + "\n");

  // Init ESP-NOW
  logToAll("setting up ESPNOW\n");
  // Set device as a Wi-Fi Station and set channel
  //WiFi.mode(WIFI_STA);
  logToAll("ESP local MAC addr: " + String(WiFi.macAddress()) + "\n");
  channel = WiFi.channel();
  logToAll("ESP wifi channel: " + String(channel) + "\n");
  // TBD read saved channel from preferences

  if (int err = esp_now_init() != ESP_OK) {
    logToAll("Error initializing ESP-NOW: " + String(err) + "\n");
  } else
    logToAll("ESP-NOW initialized\n");

  //esp_now_register_send_cb(OnDataSent);
  //esp_now_register_recv_cb(OnDataRecv);
  pairingStatus = PAIR_REQUEST;
/*
  // Register peer
  memcpy(peerInfo.peer_addr, serverAddress, 6);
  peerInfo.encrypt = false;
  peerInfo.channel = 0;
//#ifdef DEBUG
    logToAll("ESP peer MAC addr: ");
    char buf[128]; String sBuf;
    for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
      sprintf(buf, "%02X ", peerInfo.peer_addr[i]);
      sBuf += buf;
    }
    logToAll(sBuf + "\n");
    logToAll("channel: " + String(peerInfo.channel) + " ifidx: " + String(peerInfo.ifidx) + " encrypt: " + String(peerInfo.encrypt) + "\n");
//#endif  
  // Add peer
  if (err = esp_now_add_peer(&peerInfo) != ESP_OK) {
    logToAll("Failed to add peer: " + String(err) + "\n");
  } else
    logToAll("ESP peer added\n");
*/
#ifdef N2K
  setupN2K();
#endif
  logToAll("compass check frequency: " + String(compassParams.frequency) + "\n");
  consLog.close();

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    if (compassReady) {
      // Send Events to the client with the Sensor Readings Every x msecs
      events.send("ping",NULL,millis());
      events.send(getSensorReadings().c_str(),"new_readings" ,millis());
      WebSerial.printf("heading: %.2f degrees, accuracy %.2f/%.2f r/d, cal status %d\n", heading, accuracy, accuracy*180.0/M_PI, calStatus);
      //Serial.printf("heading: %.2f degrees, accuracy %.2f/%.2f r/d, cal status %d\n", heading, accuracy, accuracy*180.0/M_PI, calStatus);
    }
  });

  app.onRepeat(compassParams.frequency, []() {
    heading = getCompassHeading(compassParams.variation, compassParams.orientation);
#ifdef N2K
    SendN2kCompass(heading);
#endif
  });  

  // update web page 4x per second
  app.onRepeat(250, []() {
    events.send(getSensorReadings().c_str(), "new_readings", millis());
  });

  app.onRepeat(100, []() {
    if (autoPairing() == PAIR_PAIRED)
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) host.c_str(), host.length());
#ifdef N2K
    PollCANStatus();
#endif
    drd->loop(); // double reset detector
    WebSerial.loop();
    ElegantOTA.loop();  
  });

// if wifi not connected, we're only going to attempt reconnect once every 5 minutes
// if we get in range, it's simpler to reboot than to constantly check
  app.onRepeat(300000, []() {
    check_status(); // wifi
  });

#ifdef SH_ESP32
  // update results
  app.onRepeat(1000, []() {
    display->clearDisplay();
    //display->setRotation(2);
    display->setTextSize(1);
    display->setCursor(0, 0);
    display->setTextColor(SSD1306_WHITE);
#ifdef N2K
    display->printf("CAN: %s\n", can_state.substring(0, 3).c_str());
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
    //num_n2k_messages = 0;
  });
#endif

} // setup

// with ReactESP everything is scheduled through app lambdas
void loop() { app.tick(); }
