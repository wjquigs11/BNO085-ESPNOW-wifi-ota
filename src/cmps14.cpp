#ifdef CMPS14
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

extern int CMPS14_ADDRESS;
#define ANGLE_8  1          // Register to read 8bit angle from (starting...we read 5 bytes)
#define CAL_STATE 0x1E      // register to read calibration state
byte calibrationStatus[8];
bool readCalibrationStatus();

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8;
float comp16;
#define VARIATION -15.2
static int variation;
float mastCompassDeg; 
float boatCompassDeg, boatAccuracy, boatCompassPi;
int boatCalStatus;
float mastDelta;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
int mastOrientation, sensOrientation, boatOrientation;
extern int compassFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

extern bool compassOnToggle;
bool teleplot;
extern compass_s compassParams;

// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
// TBD: make this object oriented and overload getCompass for either CMPS14 or BNO085
//float getCMPS14(int correction) {
float getCompassHeading(int variation, int orientation) {
  //Serial.println("getCompass");
  Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
  Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
  Wire.endTransmission();
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS14_ADDRESS, 5); 
  while((Wire.available() < 5)); // (this can hang?)
  angle8 = Wire.read();               // Read back the 5 bytes
  comp8 = map(angle8, 0, 255, 0, 359);
  //comp8 = (comp8 + orientation + 360) % 360;
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  angle16 = (high_byte <<8) + low_byte;                 // Calculate 16 bit angle
  comp16 = (angle16/10) + (angle16%10)/10.0;
  comp16 += orientation;
#ifdef DEBUG
    Serial.print("angle8: "); Serial.print(angle8, DEC);
    Serial.print(" comp8: "); Serial.print(comp8, DEC);
    Serial.print(" angle 16: ");     // Display 16 bit angle with decimal place
    Serial.print(angle16/10, DEC);
    Serial.print(".");
    Serial.print(angle16%10, DEC);
    Serial.print(" a16: "); Serial.printf("%0.2f", (angle16/10) + (angle16%10)/10.0);
    Serial.print(" corr: "); Serial.println(orientation);
#endif
  if (comp16 > 359) comp16 -= 360;
  if (comp16 < 0) comp16 += 360;
  if (teleplot) {
    Serial.print(">mast: "); Serial.println(comp16); 
  }
  Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
  Wire.write(CAL_STATE);                     // Sends the register we wish to start reading from
  Wire.endTransmission();
  Wire.requestFrom(CMPS14_ADDRESS, 5); 
  while((Wire.available() < 1)); // (this can hang)
  boatCalStatus = Wire.read() & 0x3;
  readCalibrationStatus();
  ////Serial.printf("calStatus: 0x%x\n", boatCalStatus);
  //logToAll("heading: " + String(comp16) + " cal stat: " + String(boatCalStatus));
  return comp16;
}

bool readCalibrationStatus() {
  ////Serial.print("read cal status");
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CAL_STATE);
  int nackCatcher = Wire.endTransmission();
  if (nackCatcher != 0) return false;
  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(CMPS14_ADDRESS, 1);
  if (nReceived != 1) return false;
  byte Byte = Wire.read();
  for (int i = 0; i < 8; i++) {
    bool b = Byte & 0x80;
    if (b) {
      calibrationStatus[i] = 1;
    } else {
      calibrationStatus[i] = 0;
    }
    Byte = Byte << 1;
  }
  return true;
}
#endif // CMPS14