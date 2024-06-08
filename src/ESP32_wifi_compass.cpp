
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
//#include "ThingSpeak.h"
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

File consLog;

#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool compassFound = false;
// yellow = SCL = 22
// blue = SDA = 21

float lastValue=0;

#define VARIATION -15.2
static int variation=VARIATION;
static int orientation; // how is the compass oriented on the board

Preferences preferences;     

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted=false;
AsyncEventSource events("/events");
extern void setupWifi();
extern void startWebServer();
String host = "ESPcompass";
JSONVar readings;

// Timer variables
#define DEFDELAY 1000
unsigned long lastTime;
unsigned long displayDelay=DEFDELAY;
unsigned long timerDelay=DEFDELAY;
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

// TBD: put these in a shared header file
typedef struct compass_s {
    int id;
    float heading;
    float accuracy;
    int calStatus;
    int readingId;
} compass_s;
compass_s compassData;

// struct we will get from controller
// not using orientation right now since we're correcting for it in the main controller code
typedef struct control_s {
  bool compassOnToggle = true;
  int orientation = 0;
  int frequency = 500;
} control_s;
control_s inCommand;

unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
#define BNOREADRATE 20  // msecs for 50Hz rate; optimum for BNO08x calibration

// ThingSpeak
//#define THINGSPEAK
#ifdef THINGSPEAK
unsigned long myChannelNumber = 1;
const char *myWriteAPIKey = "KK4LTOKWLF5VFX0I";
WiFiClient client;
#define TSDELAY 15000 // max update rate 15 secs
unsigned long previousTS;
#endif

// Insert your SSID
//constexpr char WIFI_SSID[] = "dmlgoo";
int32_t getWiFiChannel(const char *ssid);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

//#define N2K
void SendN2kCompass(float heading);
void setupN2K();

// BNO: define the sensor outputs we want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (gameRot)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
      Serial.println("Could not enable game vector");
    else Serial.println("enabled game vector");
  if (absRot)
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR,100))
      Serial.println("Could not enable rotation vector");
    else Serial.println("enabled abs rotation vector");
#if NOTDEF
  if (!bno08x.enableReport(SH2_PRESSURE,5000))
    Serial.println("Could not enable pressure");
  else Serial.println("enabled pressure");
  if (!bno08x.enableReport(SH2_HUMIDITY,5000))
    Serial.println("Could not enable humidity");
  else Serial.println("enabled humidity");
  if (!bno08x.enableReport(SH2_TEMPERATURE,5000))
    Serial.println("Could not enable temperature");
  else Serial.println("enabled temperature");
#endif
}

float getMastHeading() {
  if (!compassFound) return -1.0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousReading < BNOREADRATE)
    return -2.0; // minimum delay in case displayDelay is set too low
  previousReading = currentMillis;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  //printf("ID: %d\n", sensorValue.sensorId);
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
        heading = calculateHeading(sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k, variation+orientation);
        printf("game vector %.2f %.2f %.2f %.2f ", sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
        printf("heading: %.2f deg\n", heading);
        return heading;
      }
      break;
    case SH2_ROTATION_VECTOR:
      if (absRot) {
        accuracy = sensorValue.un.rotationVector.accuracy;
        heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);
//        Serial.printf("heading1: %.2f ", heading);
//        heading2 = calculateHeading2(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);
//        printf("rota vector status %d %.2f %.2f %.2f %.2f ", calStatus, sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
//        printf("heading2: %.2f degrees, accuracy %.2f/%.2f r/d, status %d\n", heading2, accuracy, accuracy * 180.0 / M_PI, calStatus);
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
    printf("difference %.2f\n", abs(heading-heading2));
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
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}

float calculateHeading2(float r, float i, float j, float k, int correction) {
  float heading = atan2(2.0 * (i * j + k * r), r * r - i * i - j * j + k * k); // in radians
  heading *= (180.0 / M_PI); // convert to degrees
  heading += correction; 
  if (heading < 0) heading += 360.0;
  return heading;
}

void logToAll(String s) {
  Serial.print(s);
  consLog.print(s);
  if (serverStarted)
    WebSerial.print(s);
}
 
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  logToAll("\r\nLast Packet Send Status:\t");
  logToAll(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success: " : "Delivery Fail: ");
  logToAll(String(status) + "\n");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  logToAll("Packet received: ");
#define DEBUG
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
  }
  else {
    logToAll("done\n");
  }
}

void WebSerialonMessage(uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i = 0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
    if (d.equals("spiffs")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
    }
    if (d.equals("restart")) {
      ESP.restart();
    }
    if (d.equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while(file){
        WebSerial.println(file.name());
        file.close(); // Close the file after reading its name
        file = root.openNextFile();
      }
      root.close();
      WebSerial.println("done");
    }
    if (d.equals("scan")) {
      i2cScan();
    }
}

void setup() {
  Serial.begin(115200); delay(300);
  //Wire.begin();
  if (SPIFFS.begin())
    Serial.println("opened SPIFFS");
  else
    Serial.println("failed to open SPIFFS");
  consLog = SPIFFS.open("/console.log", "w", true);
  if(!consLog)
    Serial.println("failed to open console log");
  if(consLog.println("ESP compass console log.")) {
    Serial.println("console log written");
  } else {
    Serial.println("console log write failed");
  }
  logToAll("Adafruit BNO08x based tilt-compensated compass\n");

  setupWifi();
  if (!MDNS.begin(host.c_str()) )
    logToAll(F("Error starting MDNS responder\n"));
  else {
    logToAll("MDNS started " + host + "\n");
  }
  //Serial.printf("MDNS started %s\n", host.c_str());

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT))
    logToAll("MDNS add service failed\n");
    
  startWebServer();

  ElegantOTA.begin(&server);

  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hello Billy.");
  });

  // Initialize WebSerial
  WebSerial.begin(&server);
  delay(1000);

  // Attach a callback function to handle incoming messages
  WebSerial.onMessage(WebSerialonMessage);

  server.begin();
  serverStarted = true;
  logToAll("HTTP server started @" + String(WiFi.localIP()) + "\n");

  preferences.begin(host.c_str(), false);                        
  inCommand.frequency = preferences.getInt("frequency", DEFDELAY);
  variation = preferences.getInt("variation", VARIATION);  
  inCommand.orientation = preferences.getInt("orientation", 0);
  //displayDelay = preferences.getInt("displayDelay", DEFDELAY);
  gameRot = preferences.getBool("gameRot", false);
  absRot = preferences.getBool("absRot", true);

  // Try to initialize!
  compassFound = bno08x.begin_I2C(0x4a);
  if (!compassFound) {
    logToAll("Failed to find BNO08x chip\n");
  } else {
    logToAll("BNO08x Found\n");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) +" Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
      logToAll(logString + "\n");
    }
    setReports();
  }

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

  //Init ESP-NOW
  if (err=esp_now_init() != ESP_OK) {
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
/*
  logToAll("ESP peer MAC addr: ");
  char buf[128]; String sBuf;
  for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
    sprintf(buf, "%02X ", peerInfo.peer_addr[i]);
    sBuf += buf;
  }
  logToAll(sBuf + "\n");
  logToAll("channel: " + String(peerInfo.channel) + " ifidx: " + String(peerInfo.ifidx) + " encrypt: " + String(peerInfo.encrypt) + "\n");
*/
  // Add peer        
  if (err=esp_now_add_peer(&peerInfo) != ESP_OK){
    logToAll("Failed to add peer: " + String(err) + "\n");
  } else logToAll("ESP peer added\n");
#ifdef N2K
  setupN2K();
#endif
#ifdef THINGSPEAK
  ThingSpeak.begin(client);
#endif
  logToAll("display frequency: " + String(inCommand.frequency) + "\n");
  consLog.flush();
} // setup

extern void check_status();
String getSensorReadings();
extern DoubleResetDetector* drd;

// TBD: change to ReactESP so it's obvious that display frequency does not need to match transmit frequency
void loop() {
  drd->loop();
  ElegantOTA.loop();
  unsigned long currentMillis = millis();
  // sending on perhaps different schedule i.e. more frequently than we want to report on serial
  //Serial.printf("%d %ld %ld %d\n", inCommand.compassOnToggle, currentMillis, previousMillis, inCommand.frequency);
  if (inCommand.compassOnToggle && (currentMillis - previousMillis > inCommand.frequency)) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    float heading = getMastHeading();
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
#ifdef N2K
    SendN2kCompass(heading);
#endif
#ifdef THINGSPEAK
  if (currentMillis - previousTS > TSDELAY) {
    previousTS = currentMillis;
    int x = ThingSpeak.writeField(myChannelNumber, 1, heading, myWriteAPIKey);
    if (x == 200)
      Serial.println("Channel update successful.");
    else
      Serial.println("Problem updating channel. HTTP error code " + String(x));
      //int y = ThingSpeak.getLastReadStatus();
  }
#endif
    //Serial.printf("c-p %ld %ld %d %d ", currentMillis, previousMillis, currentMillis-previousMillis, displayDelay);
    if (currentMillis - previousDisplay > displayDelay) {
      previousDisplay = currentMillis;
      WebSerial.printf("heading: %.2f degrees, accuracy %.2f/%.2f r/d, cal status %d\n", heading, accuracy, accuracy * 180.0 / M_PI, calStatus);
    }
  }
  check_status();  
  WebSerial.loop();
}


    /* only if we go back to ESPNOW for data
    compassData.id = BOARD_ID;
    compassData.heading = heading;
    compassData.accuracy = accuracy;
    compassData.calStatus = calStatus;
    compassData.readingId = readingId++;
    */
    /* Send message via ESP-NOW
    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &compassData, sizeof(compassData));
    if (result == ESP_OK) {
      //Serial.printf("heading: %.2f degrees, accuracy %.2f/%.2f r/d, cal status %d\n", heading, accuracy, accuracy * 180.0 / M_PI, calStatus);
    } else {
      Serial.printf("Error sending the data: %d\n", result);
    }
    */

