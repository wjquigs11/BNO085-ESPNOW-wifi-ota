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

compass_s compassParams;  // we use this even with no ESPNOW
extern sh2_SensorValue_t sensorValue;

#ifdef ESPNOW
unsigned int readingId = 0;
extern Preferences preferences;

// ESPNOW
// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
// MAC Address of the other side 
uint8_t serverAddress[] = {0xC8, 0x2E, 0x18, 0xEF, 0xFC, 0xD0}; // now set from broadcast

esp_now_peer_info_t peerInfo;
bool foundPeer = false;
int channel;

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

int sendgood, sendbad;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast Packet Send Status:\t");
    if (status == ESP_NOW_SEND_SUCCESS) sendgood++;
      else sendbad++;
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    //Serial.printf("sendgood: %d, sendbad: %d, rate: %0.2f\n", sendgood, sendbad, (float)(sendgood/(sendgood+sendbad)));
    float rate = ((float)sendgood/(sendgood+sendbad));
    if ((sendgood+sendbad) % 100 == 1) {
      logToAll("sendgood: " + String(sendgood) + " sendbad: " + String(sendbad) + " rate: " + String(rate));
    }
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
    memcpy(serverAddress, mac_addr, ESP_NOW_ETH_ALEN);
    memcpy(peerInfo.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    peerInfo.encrypt = false;
    peerInfo.channel = channel;
    peerInfoS = "channel: " + String(peerInfo.channel) 
                      + " ifidx: " + String(peerInfo.ifidx) 
                      + " encrypt: " + String(peerInfo.encrypt);
    logToAll(peerInfoS);
    peerInfoS = String();
    // Add peer  
    int err;      
    if (err=esp_now_add_peer(&peerInfo) != ESP_OK) {  // succeeds even if peer_addr is null
        logToAll("Failed to add peer: " + err);
        return;
    } else logToAll("added ESPNOW peer");
    foundPeer = true;
    int b;
    b = preferences.putBytes("peerMac", mac_addr, ESP_NOW_ETH_ALEN);
    if (b == 0)
      logToAll("failed to store peer MAC in preferences");
    else {
      char peerMAC[ESP_NOW_ETH_ALEN];
      logToAll("stored peer MAC in preferences");
      b = preferences.getBytes("peerMAC", peerMAC, ESP_NOW_ETH_ALEN);
      logToAll("retrieved peer MAC from preferences" + String(b));
    }
  } // else do something with received data after peering
}
 
void setupESPNOW(const char *ssid) {
    Serial.println("setting up ESPNOW");

    // Set device as a Wi-Fi Station and set channel
    WiFi.mode(WIFI_STA);
    //WiFi.disconnect(); // for strange reasons WiFi.disconnect() makes ESP-NOW work
    //Serial.println("WiFi.disconnect() done");

    Serial.print("ESP local MAC addr: ");
    Serial.println(WiFi.macAddress());

    channel = getWiFiChannel(ssid);
    int err;

    WiFi.printDiag(Serial); // Uncomment to verify channel number before
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
    if (preferences.isKey("peerMAC")) {
      preferences.getBytes("peerMAC", serverAddress, ESP_NOW_ETH_ALEN);
      foundPeer = true;
      logToAll("found peer from preferences");
    } else
      logToAll("no peer found in preferences");
    // TBD need to add logic in case either side changes hardware
}

void loopESPNOWcontrol() {
    compassParams.id = BOARD_ID;
    compassParams.heading = 100;
    compassParams.accuracy = 1;
    compassParams.calStatus = 1;
    compassParams.readingId = readingId++;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &compassParams, sizeof(compassParams));
    if (result == ESP_OK) {
      logToAll("ESPNOW Sent with success");
    } else {
      logToAll("ESPNOW Error sending the data: " + String(result));
    }
}

void loopESPNOW() {
  if (foundPeer) {
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