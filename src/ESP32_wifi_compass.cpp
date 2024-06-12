
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
// #include "ThingSpeak.h"
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ReactESP.h>

File consLog;
using namespace reactesp;
ReactESP app;

#define BNO08X_RESET 19
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool compassFound = false;
#define ADABNO 0x4A
#define SPARKBNO 0x4B
// #define SCL 22 // yellow
// #define SDA 21 // blue

float lastValue = 0;

#define VARIATION -15.2
static int variation = VARIATION;
static int orientation; // how is the compass oriented on the board

Preferences preferences;

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted = false;
AsyncEventSource events("/events");
extern void setupWifi();
extern void startWebServer();
String host = "ESPcomp0";
JSONVar readings;
extern void check_status();
String getSensorReadings();
extern DoubleResetDetector *drd;

// Timer variables
#define DEFDELAY 50 // for compass driver update
unsigned long WebTimerDelay = 1000; // for display
bool gameRot, absRot;

float calculateHeading(float r, float i, float j, float k, int correction);
float calculateHeading2(float r, float i, float j, float k, int correction);
float heading, heading2, accuracy;
int calStatus;

// ESPNOW
#define BOARD_ID 1
// MAC Address of the controller (with boat compass)
uint8_t serverAddress[] = {0xE8, 0x6B, 0xEA, 0xD3, 0x9C, 0x7C};
esp_now_peer_info_t peerInfo;

// struct we will get from controller
// not using orientation right now since we're correcting for it in the main controller code
typedef struct control_s {
  bool compassOnToggle = true;
  int orientation = 0;
  int frequency = 100;
} control_s;
control_s inCommand;

unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
#define BNOREADRATE 20 // msecs for 50Hz rate; optimum for BNO08x calibration
int minReadRate = BNOREADRATE;

// #define N2K // if N2K defined, init CAN bus and send N2K Heading PGN

// ThingSpeak
// #define THINGSPEAK
#ifdef THINGSPEAK
unsigned long myChannelNumber = 1;
const char *myWriteAPIKey = "KK4LTOKWLF5VFX0I";
WiFiClient client;
#define TSDELAY 15000 // max update rate 15 secs
unsigned long previousTS;
#endif

int32_t getWiFiChannel(const char *ssid);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

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
float getCompassHeading() {
  if (!compassFound)
    return -1.0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousReading < BNOREADRATE) {
    logToAll("reading too soon" + String(currentMillis) + "-" + String(previousReading) + "\n");
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
  // printf("ID: %d\n", sensorValue.sensorId);
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
      heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);
      logToAll("heading1: " + String(heading) + "  cal: " + calStatus + "\n");
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
    printf("ID: %d\n", sensorValue.sensorId);
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
  // change r21 to =-2 if sensor is upside down
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
  if (heading < 0) heading += 360;
  return heading;
}

float calculateHeading2(float r, float i, float j, float k, int correction) {
  float heading = atan2(2.0 * (i * j + k * r), r * r - i * i - j * j + k * k); // in radians
  heading *= (180.0 / M_PI);                                                   // convert to degrees
  heading += correction;
  if (heading < 0)
    heading += 360.0;
  return heading;
}

// callback when ESPNOW data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  logToAll("\r\nLast Packet Send Status:\t");
  logToAll(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success: " : "Delivery Fail: ");
  logToAll(String(status) + "\n");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  logToAll("Packet received: ");
#ifdef DEBUG
  // Copies the sender mac address to a string
  char macStr[64];
  sprintf(macStr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  logToAll(String(macStr));
#endif
  memcpy(&inCommand, incomingData, sizeof(inCommand));
  logToAll(" toggle " + String(inCommand.compassOnToggle) + " orientation " + String(inCommand.orientation) + " frequency " + String(inCommand.frequency) + "\n");
  // storing here redundantly since it comes from the main controller
  preferences.putInt("orientation", inCommand.orientation);
  preferences.putInt("frequency", inCommand.frequency);
}

void i2cScan() {
  byte error, address;
  int nDevices = 0;

  logToAll("Scanning...");

  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex

    if (error == 0) {
      logToAll("I2C device found at address 0x" + String(buf) + "\n");
      nDevices++;
    }
    else if (error == 4) {
      logToAll("error at address 0x" + String(buf) + "\n");
    }
  }

  if (nDevices == 0) {
    logToAll("No I2C devices found\n");
  } else {
    logToAll("done\n");
  }
}

void WebSerialonMessage(uint8_t *data, size_t len) {
  Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  Serial.println();
  WebSerial.println("Received Data...");
  String dataS = String((char*)data);
  // Split the String into an array of Strings using spaces as delimiters
  String words[10]; // Assuming a maximum of 10 words
  int wordCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  while (endIndex != -1) {
    endIndex = dataS.indexOf(' ', startIndex);
    if (endIndex == -1) {
      words[wordCount++] = dataS.substring(startIndex);
    } else {
      words[wordCount++] = dataS.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
  }
  for (int i = 0; i < wordCount; i++) {
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        WebSerial.println(file.name());
        file.close(); // Close the file after reading its name
        file = root.openNextFile();
      }
      root.close();
      WebSerial.println("done");
    }
    if (words[i].equals("scan")) {
      i2cScan();
    }
    if (words[i].equals("hostname")) {
      host = words[++i];
      preferences.putString("hostname",host);
      logToAll("hostname set to " + host + "\n");
      logToAll("restart to change hostname\n");
    }
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin();
  // initialize compass
  compassFound = bno08x.begin_I2C(ADABNO);
  if (!compassFound) {
    logToAll("Failed to find BNO08x chip\n");
    i2cScan();
  } else {
    logToAll("BNO08x Found\n");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++)
    {
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
  inCommand.frequency = preferences.getInt("frequency", DEFDELAY);
  variation = preferences.getInt("variation", VARIATION);
  inCommand.orientation = preferences.getInt("orientation", 0);
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

  logToAll("setting up ESPNOW\n");

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  logToAll("ESP local MAC addr: " + String(WiFi.macAddress()) + "\n");

  int32_t channel = getWiFiChannel(ssid.c_str());
  int err;

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (err = esp_now_init() != ESP_OK) {
    logToAll("Error initializing ESP-NOW: " + String(err) + "\n");
  } else
    logToAll("ESP-NOW initialized\n");

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, serverAddress, 6);
  peerInfo.encrypt = false;
  peerInfo.channel = 0;
#ifdef DEBUG
    logToAll("ESP peer MAC addr: ");
    char buf[128]; String sBuf;
    for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
      sprintf(buf, "%02X ", peerInfo.peer_addr[i]);
      sBuf += buf;
    }
    logToAll(sBuf + "\n");
    logToAll("channel: " + String(peerInfo.channel) + " ifidx: " + String(peerInfo.ifidx) + " encrypt: " + String(peerInfo.encrypt) + "\n");
#endif  
  // Add peer
  if (err = esp_now_add_peer(&peerInfo) != ESP_OK) {
    logToAll("Failed to add peer: " + String(err) + "\n");
  } else
    logToAll("ESP peer added\n");
#ifdef N2K
  setupN2K();
#endif
#ifdef THINGSPEAK
  ThingSpeak.begin(client);
#endif
  logToAll("compass check frequency: " + String(inCommand.frequency) + "\n");
  consLog.flush();

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    // Send Events to the client with the Sensor Readings Every x seconds
    events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
    WebSerial.printf("heading: %.2f degrees, accuracy %.2f/%.2f r/d, cal status %d\n", heading, accuracy, accuracy * 180.0 / M_PI, calStatus);
  });

  app.onRepeat(inCommand.frequency, []() {
    heading = getCompassHeading();
    events.send(getSensorReadings().c_str(), "new_readings", millis());
#ifdef N2K
    SendN2kCompass(heading);
#endif
  });

#ifdef THINGSPEAK
  app.onRepeat(TSDELAY, []() {
    int x = ThingSpeak.writeField(myChannelNumber, 1, heading, myWriteAPIKey);
    if (x == 200)
      Serial.println("Channel update successful.");
    else
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    // int y = ThingSpeak.getLastReadStatus();
  }
#endif

// functions like loop(); twice per second do maintenance tasks
  app.onRepeat(500, []() {
    // double reset detection
    drd->loop();
    // OTA update check
    ElegantOTA.loop();
    // check wifi status
    check_status();
    // check for commands on WebSerial  
    WebSerial.loop();
  });

} // setup

// with ReactESP everything is scheduled through app lambdas
void loop() { app.tick(); }
