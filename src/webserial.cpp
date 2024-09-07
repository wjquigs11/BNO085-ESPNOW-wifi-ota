#include <Arduino.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WebSerial.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include "compass.h"

extern String host;
extern JSONVar readings;
static int orientation;
extern Preferences preferences;
extern unsigned long WebTimerDelay;
extern int num_n2k_sent;
extern int calStatus;

#ifdef N2K
extern int num_n2k_sent, num_n2k_messages;
extern String can_state;
#endif

extern float heading, accuracy;

void logToAll(String s);
#define PRBUF 128
char prbuf[PRBUF];

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;

  logToAll("Scanning...");

  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    sprintf(prbuf, "%2X", address); // Formats value as uppercase hex

    if (error == 0) {
      logToAll("I2C device found at address 0x" + String(prbuf) + "\n");
      nDevices++;
    }
    else if (error == 4) {
      logToAll("error at address 0x" + String(prbuf) + "\n");
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
    WebSerial.println(words[i]);
    if (words[i].equals("?")) {
      WebSerial.println("restart");
      WebSerial.println("format");
      WebSerial.println("ls");
      WebSerial.println("scan (i2c)");
      WebSerial.println("hostname");
      WebSerial.println("status");
      WebSerial.println("wificonfig");
      return;
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
      return;
    }
    if (words[i].equals("restart")) {
      WebSerial.println("restarting...");
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
      return;
    }
    if (words[i].equals("scan")) {
      //i2cScan();
      return;
    }
    if (words[i].equals("hostname")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
        logToAll("hostname set to " + host + "\n");
        logToAll("restart to change hostname\n");
        logToAll("preferences " + preferences.getString("hostname") + "\n");
      } else {
        logToAll("hostname: " + host + "\n");
      }
      return;
    }
    if (words[i].equals("status")) {
      String buf = "hostname: " + host + ", variation: " + String(compassParams.variation) + ", orientation: " + String(compassParams.orientation) + ", timerdelay: " + String(WebTimerDelay);
      buf += ", frequency: " + String(compassParams.frequency);
      logToAll(buf);
#ifdef N2K
      buf = "n2k xmit: " + String(num_n2k_sent);
      buf += ", n2k recv: " + String(num_n2k_messages);
      buf += ", can state: " + can_state;
      logToAll(buf);
#endif
      sprintf(prbuf, "heading: %.2f d, %0.2f r, accuracy %.2f/%.2f r/d, cal status %d\n", heading, heading*DEGTORAD, accuracy, accuracy*180.0/M_PI, calStatus);
      logToAll(prbuf);
      buf = "wifi: " + WiFi.SSID();
      logToAll(buf);
      logToAll(JSON.stringify(readings));
      buf = String();
      return;
    }
    logToAll("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
