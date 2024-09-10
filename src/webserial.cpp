#include <Arduino.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WebSerial.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <esp_now.h>

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

#ifdef ESPNOW
extern uint8_t serverAddress[];
extern bool foundPeer;
extern bool espnowtoggle;
extern int totalReports;
extern int goodReports;
#endif
extern int reportType;
extern Adafruit_BNO08x bno08x;

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

String formatMacAddress(const String& macAddress) {
  String result = "{";
  int len = macAddress.length();
  
  for (int i = 0; i < len; i += 3) {
    if (i > 0) {
      result += ", ";
    }
    result += "0x" + macAddress.substring(i, i + 2);
  }
  
  result += "};";
  return result;
}

String commandList[] = {"?", "format", "restart", "ls", "scan", "hostname", "status", "wificonfig", "espnowtoggle"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))
String words[10]; // Assuming a maximum of 10 words

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
    int j;
    WebSerial.println(words[i]);
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        WebSerial.println(String(j) + ":" + commandList[j]);
      }
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
      logToAll(JSON.stringify(readings));
#ifdef ESPNOW
      logToAll("compass reports total: " + String(totalReports) + " good: " + String(goodReports));
      logToAll("espnowtoggle " + String(espnowtoggle));
#endif
      logToAll("reportType " + String(reportType,HEX));
      WebSerial.flush();
      buf = String();
      return;
    }
    if (words[i].equals("wificonfig")) {
      String buf = "hostname: " + host;
      buf += " wifi: " + WiFi.SSID();
      buf += " ip: " + WiFi.localIP().toString();
      buf += "  MAC addr: " + formatMacAddress(WiFi.macAddress());
      logToAll(buf);
#ifdef ESPNOW
      if (foundPeer) {
        buf = "peer: ";
        for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) { 
          if (i > 0) buf += ":";
          buf += String(serverAddress[i], HEX);
          if (buf.length() == 1) buf = "0" + buf;
        } 
        logToAll(buf);
      } else logToAll("no ESPNOW peer");
#endif
      buf = String();
      return;
    }
#ifdef ESPNOW
    if (words[i].equals("espnowtoggle")) {
      espnowtoggle = !espnowtoggle;
      logToAll("espnowtoggle " + String(espnowtoggle));
      return;
    }
#endif
    if (words[i].equals("rtype")) {
      reportType = preferences.getInt("rtype", 0); // change after 1st write
      if (!words[++i].isEmpty()) {
        reportType = (int)strtol(words[i].c_str(), NULL, 16);
        preferences.putInt("rtype", reportType);
        WebSerial.printf("compass report type set to 0x%x\n", reportType);
        if (!bno08x.enableReport(reportType))
              WebSerial.printf("Could not enable local report 0x%x\n",reportType);
      } else {
        WebSerial.printf("compass report type is 0x%x\n",reportType);
      }
      return;
    }
    logToAll("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
