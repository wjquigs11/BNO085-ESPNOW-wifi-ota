
#include <Arduino.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <math.h>
#include <esp_wifi.h>
#include "Async_ConfigOnDoubleReset_Multi.h"
//#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
//#include <WebSerial.h>
#include <ReactESP.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include "elapsedMillis.h"
#include <driver/adc.h>
#include <esp_timer.h>
#include <movingAvg.h>
#include "soc/rtc.h"
#include <HX711_ADC.h>
#include <EEPROM.h>

File consLog, tachLog;
using namespace reactesp;
ReactESP app;

float lastValue = 0;

Preferences preferences;

// HX711 circuit wiring
const int HX711_dout = 18; //mcu > HX711 dout pin
const int HX711_sck = 4; //mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 4;
unsigned long t = 0;
float calibrationValue; // calibration value (see example file "Calibration.ino")

#define EMPTY_TANK_2 1669.21362

#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
bool serverStarted = false;
AsyncEventSource events("/events");
extern void setupWifi();
extern void startWebServer();
String host = "ESPtach";
JSONVar readings;
extern void check_status();
String getSensorReadings();
extern DoubleResetDetector *drd;

//#define SH_ESP32 // the only version with a display
#ifdef SH_ESP32
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
TwoWire *i2c;
// for SPI BNO compass
#define RESET 14 // pull low to reset
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

// timing
#define DEFDELAY 50 // for compass driver update
unsigned long WebTimerDelay = 1000; // for display
unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
unsigned long currentMillis = millis();

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void logToAll(String s) {
  Serial.println(s);
  consLog.println(s);
//  if (serverStarted)
//    WebSerial.println(s);
}

//void WebSerialonMessage(uint8_t *data, size_t len);

#define ADCPIN 34 // GPIO34 (ADC1_CH6)
int count, countPerSecond, adcLow=9999, adcHigh=-1, adcAverage, runningTotal, numsamples, newAvg;
#define THRESHOLD 2000  // ADC value threshold (adjust as needed)
#define SAMPLE_RATE 1000  // Samples per second
// 5000 RPM is 83.33 Hz, 2-stroke one cycle per RPM
unsigned short samples[SAMPLE_RATE];
movingAvg tachometer(10);                // define the moving average object
char buf[128];

void IRAM_ATTR adcTimer(void* arg) {
  int adcValue = adc1_get_raw(ADC1_CHANNEL_6);  // Read ADC (GPIO34 is ADC1_CH6)
  if (adcValue < adcLow) adcLow = adcValue;
  if (adcValue > adcHigh) adcHigh = adcValue;
  runningTotal += adcValue;
  samples[numsamples++] = adcValue;
  if (adcValue > THRESHOLD) {
    count++;
  }
}

void IRAM_ATTR reportTimer(void* arg) {
  countPerSecond = count;
  newAvg = tachometer.reading(count);
  count = 0;
  if (numsamples)
    adcAverage = runningTotal/numsamples;
  else Serial.println("no samples");
  for (int i=0; i<numsamples; i++) {
    tachLog.print(samples[i]);
  }
  tachLog.flush();
  adcLow = 9999;
  adcHigh = -1;
  runningTotal = 0;
  numsamples = 0;
}

// zero offset value (tare), calculate and save to EEprom:
void refreshOffsetValueAndSaveToEEprom() {
  long _offset = 0;
  Serial.println("Calculating tare offset value...");
  LoadCell.tare(); // calculate the new tare / zero offset value (blocking)
  _offset = LoadCell.getTareOffset(); // get the new tare / zero offset value
  EEPROM.put(tareOffsetVal_eepromAdress, _offset); // save the new tare / zero offset value to EEprom
  EEPROM.commit();
  LoadCell.setTareOffset(_offset); // set value as library parameter (next restart it will be read from EEprom)
  Serial.print("New tare offset value:");
  Serial.print(_offset);
  Serial.print(", saved to EEprom adr:");
  Serial.println(tareOffsetVal_eepromAdress);
}

//long calibration = 134815/1306; // water bottle is 1.306kg

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("tachometer/load cell");
  tachometer.begin();

  rtc_cpu_freq_config_t config;
  rtc_clk_cpu_freq_get_config(&config);
  rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
  rtc_clk_cpu_freq_set_config_fast(&config);

  LoadCell.begin();
  //LoadCell.setReverseOutput();
  //calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch

  EEPROM.begin(512);

  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom
  Serial.printf("loaded calibration value %0.2f\n", calibrationValue);

  //restore the zero offset value from eeprom:
  long tare_offset = 0;
  EEPROM.get(tareOffsetVal_eepromAdress, tare_offset);
  Serial.printf(", loaded tare offset value %0.2ld\n", tare_offset);
  LoadCell.setTareOffset(tare_offset);
  boolean _tare = false; //set this to false as the value has been resored from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
  LoadCell.setSamplesInUse(1);
  logToAll("load cell samples: " + String(LoadCell.getSamplesInUse()));

#ifdef SH_ESP32 // use display
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
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)
  //analogSetAttenuation(ADC_11db);  // Set ADC attenuation for 3.3V full-scale range
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

#if 0
// Timer for ADC sampling
  esp_timer_create_args_t adcTimerConfig = {
    .callback = &adcTimer,
    .name = "adc_timer"
  };
  esp_timer_handle_t adcTimerHandle;
  esp_timer_create(&adcTimerConfig, &adcTimerHandle);
  esp_timer_start_periodic(adcTimerHandle, 1000000 / SAMPLE_RATE);  // Timer interval in microseconds

// Timer for reporting (every second)
  esp_timer_create_args_t reportTimerConfig = {
    .callback = &reportTimer,
    .name = "report_timer"
  };
  esp_timer_handle_t reportTimerHandle;
  esp_timer_create(&reportTimerConfig, &reportTimerHandle);
  esp_timer_start_periodic(reportTimerHandle, 1000000);  // 1 second interval
#endif
  if (SPIFFS.begin())
    Serial.println("opened SPIFFS");
  else
    Serial.println("failed to open SPIFFS");

  tachLog = SPIFFS.open("/tach.log", "w", true);
  if (!tachLog)
    Serial.println("failed to open tach log");

  // start a console.log file in case we crash before Webserial starts
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog)
    Serial.println("failed to open console log");
  if (consLog.println("ESP compass console log.")) {
    Serial.println("console log written");
  } else {
    Serial.println("console log write failed");
  }
  logToAll("tachometer/load cell\n");

  preferences.begin(host.c_str(), false);
  host = preferences.getString("host", host);
  logToAll("hostname: " + host + "\n");

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  setupWifi();
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

  startWebServer();

  //ElegantOTA.begin(&server);

  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello from Billy!"); });

  // Initialize WebSerial
  //WebSerial.begin(&server);
  // Attach a callback function to handle incoming messages
  //WebSerial.onMessage(WebSerialonMessage);

  server.begin();
  serverStarted = true;
  logToAll("HTTP server started @" + WiFi.localIP().toString() + "\n");

  consLog.flush();

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    // Send Events to the client with the Sensor Readings Every x msecs
    //events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
    //WebSerial.printf("heading: %.2f d, %0.2f r, accuracy %.2f/%.2f r/d, cal status %d\n", heading, heading*DEGTORAD, accuracy, accuracy*180.0/M_PI, calStatus);
    sprintf(buf, "Uptime: %lu\n", millis() / 1000);
    logToAll(String(buf));
    // check load cell
    static boolean newDataReady = 0;
    // check for new data/start next conversion:
    if (LoadCell.update()) newDataReady = true;
    // get smoothed value from the dataset:
    if (newDataReady) {
        float l = LoadCell.getData();
        sprintf(buf, "Load_cell output val: %0.2f", l);
        logToAll(String(buf));
        newDataReady = 0;
    }
    consLog.flush();
    sprintf(buf, "Detections per second: %d, RPM %d ", countPerSecond, countPerSecond*60);
    logToAll(String(buf));
    if (numsamples) {
      sprintf(buf, "ADC low: %d, high: %d, average: %d ", adcLow, adcHigh, runningTotal/numsamples);
      logToAll(String(buf));
    } else logToAll("no samples 2");
    sprintf(buf, "New average: %d RPM %d\n", newAvg, newAvg*60);
    logToAll(String(buf));    
    });

  app.onRepeat(10, []() {
//  WebSerial.loop();
    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') refreshOffsetValueAndSaveToEEprom();
    }
  });

  app.onRepeat(100, []() {
    drd->loop(); // double reset detector
    //ElegantOTA.loop();  
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

#if 0
void loop() {
  Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale.get_units(10), 5);

  scale.power_down();             // put the ADC in sleep mode
  delay(5000);
  scale.power_up();
}
#endif
