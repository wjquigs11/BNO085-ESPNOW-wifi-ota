#include "windparse.h"
#include "compass.h"
//#include <SparkFun_TMAG5273_Arduino_Library.h>
//#include "SparkFun_ISM330DHCX.h"
#include <NTPClient.h>
#include <ElegantOTA.h>
#include <Adafruit_BNO08x.h>

File consLog;
using namespace reactesp;
ReactESP app;
RepeatReaction* checkCompassReact;

float lastValue = 0;

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

float calculateHeading(float r, float i, float j, float k, int correction);

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

#ifdef D1MINI
#define BLUE SDA
#define YELLOW SCL
#endif

#ifdef BNO08X
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern int reportType;
extern int calStatus;
extern float accuracy;
//float heading;
void setReports(int rType);
#endif
compass_s compassParams;

#ifdef MASTIMU
extern SparkFun_ISM330DHCX ISM330;
TaskHandle_t mastTask;
void getISMheading(void *parameter);
bool setupMastIMU(TwoWire &wirePort, uint8_t deviceAddress);
#endif

int CMPS14_ADDRESS = 0x60;

bool compassReady = false;
float getCompassHeading(int correction);
extern float accuracy;
extern int calStatus;
extern float mastCompassDeg, mastAccuracy, mastCompassMag; 

// timing
#define DEFDELAY 50 // for compass driver update
int WebTimerDelay = 1000; // for display
unsigned long previousMillis, previousDisplay, previousReading;
//int minReadRate = BNOREADRATE;
int minReadRate = DEFDELAY;
unsigned long currentMillis = millis();

#ifdef N2K
String can_state;
int num_n2k_sent = 0, num_n2k_messages = 0;
void SendN2kCompass(float heading);
void setupN2K();
extern tNMEA2000 *n2kesp;
#endif

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

#ifdef MQTT
#include <PicoMQTT.h>
PicoMQTT::Server mqtt;
#endif

#ifdef HALLSENS
void initHall();
int hallLoop();
void sensHall();
extern int hallValue;
extern bool hallTrigger;
extern elapsedMillis time_since_last_hall_adjust;
#define HALL_DELAY 500  // don't update compass difference more than twice per second
#define HALL_LIMIT 10   // arbitrary limit for magnet near sensor; adjust as needed
#endif

bool teleplot = true;

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

void i2cScan(TwoWire Wire);
void WebSerialonMessage(uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("I2C compass");
  Wire.setClock(100000);
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
#define yellow SCL
#define blue SDA
#ifdef BNO08X
  Wire1.begin(SDA,SCL);
  if (!(compassReady = bno08x.begin_I2C(0x4B, &Wire1, 0)))
    i2cScan(Wire1);
#else
  Wire.setClock(100000);  // slow clock to compensate for errors
  Wire.setTimeOut(1000); // Set timeout to 1 second
  compassReady = bno08x.begin_I2C(0x4A, &Wire, 0);
  if (!compassReady) {
    Serial.println("BNO08x not found");
    i2cScan(Wire);
  }
#endif
  if (compassReady) {
    Serial.println("BNO08X present");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
      logToAll(logString);
    }
    setReports(reportType);
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
  logToAll("tilt-compensated compass");

  preferences.begin("ESPcompass", false);
  compassParams.frequency = preferences.getInt("frequency", DEFDELAY);
  if (compassParams.frequency < minReadRate) compassParams.frequency = minReadRate;
  if (compassParams.frequency > 1000) compassParams.frequency = 1000;
  logToAll("frequency " + String(compassParams.frequency));
  //Serial.printf("frequency %d\n", compassParams.frequency);
  compassParams.variation = preferences.getInt("variation", VARIATION);
  logToAll("variation " + String(compassParams.variation));
  //Serial.printf("variation %d\n", compassParams.variation);
  compassParams.orientation = preferences.getInt("orientation", 0);
  logToAll("orientation " + String(compassParams.orientation));
  //Serial.printf("orientation %d\n", compassParams.orientation);
  WebTimerDelay = preferences.getInt("timerdelay", 1000);
  if (WebTimerDelay<200) {
    WebTimerDelay = 200;
    preferences.putInt("timerdelay", 200);
  }
  logToAll("WebTimerDelay " + String(WebTimerDelay));
  //Serial.printf("WebTimerDelay %d\n", WebTimerDelay);
  host = preferences.getString("hostname", host);
  logToAll("hostname: " + host + "\n");
#ifdef BNO08X
  reportType = preferences.getInt("rtype", SH2_GAME_ROTATION_VECTOR);
  logToAll("reportType = 0x" + String(reportType,HEX));
#endif
  if (!MDNS.begin(host.c_str()))
    logToAll(F("Error starting MDNS responder"));
  else {
    logToAll("MDNS started " + host);
  }

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT))
    logToAll("MDNS add service failed");

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
  int retries;
  while(!timeClient.update() && (retries++ < 10)) {
      Serial.print(".");
      timeClient.forceUpdate();
  }
  Serial.println();

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

#ifdef MQTT
  // start MQTT broker
    mqtt.begin();
#endif

  consLog.flush();

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    static int counter;
    if (compassReady) {
      // Send Events to the client with the Sensor Readings Every x msecs
      //events.send("ping",NULL,millis());
      events.send(getSensorReadings().c_str(),"new_readings" ,millis());
      //WebSerial.printf("heading: %.2f d, %0.2f r, accuracy %.2f/%.2f r/d, cal status %d\n", heading, heading*DEGTORAD, accuracy, accuracy*180.0/M_PI, calStatus);
    }
    consLog.flush();
});

  checkCompassReact = app.onRepeat(compassParams.frequency, []() {
#ifndef MASTIMU // mastCompassDeg is set in the ISM330 Mahony task
    if (compassReady)
      mastCompassDeg = getCompassHeading(compassParams.variation+compassParams.orientation);
    else mastCompassDeg = -1;
    if (mastCompassDeg <-3) logToAll("heading error: " + String(mastCompassDeg));
#endif
#ifdef MQTT
    mqtt.publish("mastrotate/mastcompass", String(mastCompassDeg,2));
#endif
  });  

#ifdef HALLSENS
  app.onRepeat(10, []() {
    sensHall();
  });
#endif

#ifdef N2K
  app.onRepeat(10, []() {
    //PollCANStatus();
    n2kesp->ParseMessages();
    SendN2kCompass(mastCompassDeg);
  });
#endif

  app.onRepeat(10, []() {
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
    display->printf("IMU:\n");
    display->setTextSize(2);
    display->printf("%.1f\n", mastCompassDeg);
    display->setTextSize(1);
    display->printf("Mag:\n");
    display->setTextSize(2);
    display->printf("%.1f\n", mastCompassMag);
    display->setTextSize(1);
    display->printf("%.1f\n", accuracy*RADTODEG);
    display->printf("%d\n", calStatus);
    display->display();
#ifdef N2K
    num_n2k_sent = num_n2k_messages = 0;
#endif
  });
#endif  // SH_ESP32
}

// with ReactESP everything is scheduled through app lambdas
void loop() { app.tick(); }
