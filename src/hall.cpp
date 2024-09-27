#ifdef HALLSENS
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
#include "SparkFun_TMAG5273_Arduino_Library.h"

#include "compass.h"

TMAG5273 hallSensor; //  hall-effect sensor
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

void initHall();
int hallLoop();
int hallValue;
bool hallTrigger = false;
bool hallDisplay = false;
elapsedMillis time_since_last_hall_adjust = 0;
#define HALL_DELAY 500  // don't update compass difference more than twice per second
#define HALL_LIMIT 10   // arbitrary limit for magnet near sensor; adjust as needed

extern bool teleplot;
extern float heading;

void logToAll(String s);
void i2cScan(TwoWire Wire);


int hallLoop() {
  int stat = hallSensor.getDeviceStatus();
  //Serial.printf("hall status %d\n", stat);
  if (hallSensor.getMagneticChannel() != 0) {
    float magX = hallSensor.getXData();
    float magY = hallSensor.getYData();
    float magZ = hallSensor.getZData();
    float temp = hallSensor.getTemp()*9/5+32;
    int totalMag = abs(magX) + abs(magY) + abs(magZ);
    if (teleplot && 0) {
      Serial.print(">magX:");
      Serial.println(magX);
      Serial.print(">magY:");
      Serial.println(magY);
      Serial.print(">magZ:");
      Serial.println(magZ);
      Serial.println("temp:");
      Serial.println(temp);
      Serial.print(">totalMag:");
      Serial.println(totalMag);
    }
    return totalMag;
  } else {
    return -1;
  }
}

void initHall() {
  int err;
  if((err = hallSensor.begin(i2cAddress, Wire)) == 1) {
    logToAll("started Hall Effect sensor");
    if (hallLoop()<0) {
      logToAll("Mag Channels disabled");
    } else hallSensor.setTemperatureEn(true);
  } else {
    logToAll("failed to start Hall Effect sensor " + String(err));
    i2cScan(Wire);
  }
}

void sensHall() {
    // when Hall effect sensor is triggered, mast is centered
    // we can change mastorientation to the difference between the two IMU readings
    if ((hallValue = hallLoop()) > HALL_LIMIT) {
        if (time_since_last_hall_adjust > HALL_DELAY) {
        logToAll("Hall effect detected, mast (raw): " + String(heading,2) + " hallvalue: " + hallValue);
        // set flag for next N2K transmit
        hallTrigger = true;
        hallDisplay = true;
        time_since_last_hall_adjust = 0;
        }
    }
}
#endif