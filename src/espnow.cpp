#include <Arduino.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "compass.h"

void logToAll(String s);

extern sh2_SensorValue_t sensorValue;

#ifdef ESPNOW
unsigned int readingId = 0;
extern Preferences preferences;
bool espnowtoggle;

// ESPNOW
// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
// MAC Address of the other side 
uint8_t serverAddress[] = {0xC8, 0x2E, 0x18, 0xEF, 0xFC, 0xD0}; // now set from broadcast

esp_now_peer_info_t peerInfo;
bool foundPeer = false;
int channel;
extern compass_s compassParams;
int sendgood, sendbad;
int reportType; // which report do we want compass to generate? set by main processor (espwind)
void setReports();

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

bool sendControl() {
    //compassParams.id = BOARD_ID;
    compassParams.heading = 100;
    compassParams.accuracy = 1;
    compassParams.calStatus = 1;
    compassParams.readingId = readingId++;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &compassParams, sizeof(compassParams));
    if (result == ESP_OK) {
      logToAll("ESPNOW control sent with success");
      return true;
    } else {
      logToAll("ESPNOW contol error sending the data: " + String(result));
      return false;
    }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast Packet Send Status:\t");
    if (status == ESP_NOW_SEND_SUCCESS) sendgood++;
      else sendbad++;
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    //Serial.printf("sendgood: %d, sendbad: %d, rate: %0.2f\n", sendgood, sendbad, (float)(sendgood/(sendgood+sendbad)));
    float rate = ((float)sendgood/(sendgood+sendbad));
    if ((sendgood+sendbad) % 100 == 1) {
      logToAll(String(millis()) + " sendgood: " + String(sendgood) + " sendbad: " + String(sendbad) + " rate: " + String(rate));
    }
}

bool registerPeer(const uint8_t * mac_addr) {
  String peerInfoS;
  memcpy(serverAddress, mac_addr, ESP_NOW_ETH_ALEN);
  memcpy(peerInfo.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
  peerInfo.encrypt = false;
  peerInfo.channel = channel;
  peerInfoS = "channel: " + String(peerInfo.channel) 
                    + " ifidx: " + String(peerInfo.ifidx) 
                    + " encrypt: " + String(peerInfo.encrypt);
  logToAll(peerInfoS);
  peerInfoS = String();
  int err;      
  if (err=esp_now_add_peer(&peerInfo) != ESP_OK) {  // succeeds even if peer_addr is null
      logToAll("registerPeer: Failed to add peer: " + err);
      return false;
  } else logToAll("registerPeer: added ESPNOW peer");
  // send ACK to peer
  return sendControl();
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  String peerInfoS = "";
  for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
      if (i > 0) peerInfoS += ":";
      peerInfoS += String(mac_addr[i], HEX);
      if (peerInfoS.length() == 1) peerInfoS = "0" + peerInfoS;
  }
  logToAll("ESPNOW got packet from: " + peerInfoS + " peer? " + String(foundPeer));
  if (!foundPeer) {
    // Register peer
    foundPeer = registerPeer(mac_addr);
    int b;
#if 0
    // store peer MAC in preferences
    logToAll(String(preferences.begin("ESPcompass", false)));
    b = preferences.putBytes("peerMac", mac_addr, ESP_NOW_ETH_ALEN);
    if (b == 0)
      logToAll("failed to store peer MAC in preferences");
    else {
      char peerMAC[ESP_NOW_ETH_ALEN];
      logToAll("stored peer MAC in preferences");
      b = preferences.getBytes("peerMAC", peerMAC, ESP_NOW_ETH_ALEN);
      logToAll("retrieved peer MAC from preferences" + String(b));
    }
#else
    for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
      b = preferences.putInt(String(i).c_str(), mac_addr[i]);
      if (b == 0) logToAll("failed to store peer MAC[] in preferences " + String(i));
    }
    for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
      b = preferences.getInt(String(i).c_str(), 0);
      logToAll("peer mac[" + String(i) + "]: " + String(b, HEX));
    }
#endif
  } // do something with received data after peering
  memcpy(&compassParams, incomingData, sizeof(compassParams));
  if (compassParams.id) {
    logToAll("setting report type to: 0x" + String(compassParams.id,HEX));
      reportType = compassParams.id;
      setReports();
  }
}
 
void setupESPNOW(const char *ssid) {
    Serial.println("setting up ESPNOW");

    // Set device as a Wi-Fi Station and set channel
    WiFi.mode(WIFI_STA);
    //WiFi.disconnect(); // for strange reasons WiFi.disconnect() makes ESP-NOW work
    //Serial.println("WiFi.disconnect() done");

    Serial.print("ESP local MAC addr: ");
    Serial.println(WiFi.macAddress());

    channel = getWiFiChannel(ssid)+2; // trying a different channel to see if we can get better tput
    int err;

    //WiFi.printDiag(Serial); // Uncomment to verify channel number before
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    WiFi.printDiag(Serial); // Uncomment to verify channel change after

    //Init ESP-NOW
    if (err=esp_now_init() != ESP_OK) {
        Serial.printf("Error initializing ESP-NOW: %d\n", err);
        return;
    } else
        Serial.println("ESP-NOW initialized");

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Transmitted packet
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    // our first received message should be a broadcast; then we will establish peering
#if 0
    if (preferences.isKey("peerMAC")) {
      preferences.getBytes("peerMAC", serverAddress, ESP_NOW_ETH_ALEN);
      foundPeer = true;
      logToAll("found peer from preferences");
    } else
      logToAll("no peer found in preferences");
#else // incredible kludge because perferences.putBytes doesn't work
    if (preferences.isKey("0")) {
      logToAll("found peer from preferences");
      char peermacS[32];
      for (int i=0; i<ESP_NOW_ETH_ALEN; i++) {
        serverAddress[i] = preferences.getInt(String(i).c_str());
      }
      sprintf(peermacS,"0x%x:0x%x:0x%x:0x%x:0x%x:0x%x",serverAddress[0],serverAddress[1],serverAddress[2],serverAddress[3],serverAddress[4],serverAddress[5]);
      logToAll("peer mac: " + String(peermacS));
    foundPeer = registerPeer(serverAddress);
    } else
      logToAll("no peer found in preferences");
#endif
    // TBD need to add logic in case either side changes hardware
}

void loopESPNOW() {
  if (foundPeer && espnowtoggle) {
    //Serial.println("sending sensor values");
    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &sensorValue, sizeof(sensorValue));
    if (result == ESP_OK) {
      //logToAll("ESPNOW Sent with success");
    } else {
      logToAll("ESPNOW Error sending the data: " + String(esp_err_to_name(result)));
    }    
  }
}
#endif