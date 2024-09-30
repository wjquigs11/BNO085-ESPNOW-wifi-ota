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
extern int WebTimerDelay;
extern int num_n2k_sent;

#ifdef N2K
extern int num_n2k_sent, num_n2k_messages;
extern String can_state;
#endif

extern float mastCompassDeg;

#ifdef BNO08X
extern int reportType;
extern Adafruit_BNO08x bno08x;
extern int IMUreports[SH2_MAX_SENSOR_ID];
extern int totalReports;
#endif

extern bool teleplot;

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

String commandList[] = {"?", "format", "restart", "ls", "scan", "hostname", "status", "wificonfig", "teleplot"};
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
    if (words[i].startsWith("host")) {
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
      unsigned long uptime = millis() / 1000;
      logToAll("uptime: " + String(uptime));
#ifdef N2K
      buf = "n2k xmit: " + String(num_n2k_sent);
      buf += ", n2k recv: " + String(num_n2k_messages);
      buf += ", can state: " + can_state;
      logToAll(buf);
#endif
      logToAll("mast compass: %0.2f" + String(mastCompassDeg,2));
      logToAll(JSON.stringify(readings));
#ifdef BNO08X
      logToAll("reportType " + String(reportType,HEX));
#endif
      buf = String();
      return;
    }
    if (words[i].startsWith("wifi")) {
      String buf = "hostname: " + host;
      buf += " wifi: " + WiFi.SSID();
      buf += " ip: " + WiFi.localIP().toString();
      buf += "  MAC addr: " + formatMacAddress(WiFi.macAddress());
      logToAll(buf);
      buf = String();
      return;
    }
    if (words[i].startsWith("teleplot")) {
      teleplot = !teleplot;
      logToAll("teleplot " + String(teleplot));
      return;
    }
  
#ifdef BNO08X
    if (words[i].startsWith("rtype")) {
      if (!words[++i].isEmpty()) {
        reportType = (int)strtol(words[i].c_str(), NULL, 16);
        preferences.putInt("rtype", reportType);
        logToAll("compass report type set to " + String(reportType));
        if (!bno08x.enableReport(reportType))
              logToAll("Could not enable local report " + String(reportType));
      } else {
        logToAll("compass report type is " + String(reportType));
        logToAll("compass reports total: " + String(totalReports));
        for (int i=0; i<SH2_MAX_SENSOR_ID; i++) {
          if (IMUreports[i])
            logToAll("IMU report " + String(i) + "/0x" + String(i, HEX) + ": " + String(IMUreports[i]));
        }
      }
      return;
    }
#endif
    if (words[i].startsWith("varia")) {
      if (!words[++i].isEmpty()) {
        compassParams.variation = atoi(words[i].c_str());
        preferences.putInt("variation", compassParams.variation);
        logToAll("variation set to " + String(compassParams.variation));
      } else {
        logToAll("variation: " + String(compassParams.variation));
      }
      return;
    }    
    if (words[i].startsWith("orient")) {
      if (!words[++i].isEmpty()) {
        compassParams.orientation = atoi(words[i].c_str());
        preferences.putInt("orientation", compassParams.orientation);
        logToAll("orientation set to " + String(compassParams.orientation));
      } else {
        logToAll("orientation: " + String(compassParams.orientation));
      }
      return;
    }
    logToAll("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
