// TBD: change params in n2k init so we're a "real" device

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <Wire.h>

#include "compass.h"

//#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1305.h>
//#include <Fonts/FreeMono9pt7b.h>
//#include <Fonts/TomThumb.h>

#ifdef SH_ESP32
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25
#endif
#ifdef D1MINI
#define CAN_RX_PIN GPIO_NUM_16
#define CAN_TX_PIN GPIO_NUM_17
#endif
#ifdef DEVKIT
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25
#endif
// leaving here in case I decide to add a display later
// Used for software or hardware SPI
//#define OLED_CS 5
//#define OLED_DC 19

// Used for I2C or SPI
//#define OLED_RESET 22

// hardware SPI - use 7Mhz (7000000UL) or lower because the screen is rated for 4MHz, or it will remain blank!
//Adafruit_SSD1305 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS, 4000000UL);

tNMEA2000 *n2kesp;
extern int num_n2k_sent, num_n2k_messages;

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127250L,0};

void logToAll(String s);

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {   
  //N2kMsg.Print(&Serial);
  //Serial.printf("t: %d R: %d\n", millis(), N2kMsg.PGN);
  num_n2k_messages++;
}

void setupN2K() {
  Serial.printf("configuring CAN bus on TX %d RX %d\n", CAN_TX_PIN, CAN_RX_PIN);
  n2kesp = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
//tNMEA2000 &N2K=*(new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz,N2k_CAN_INT_PIN,MCP_CAN_RX_BUFFER_SIZE));
  n2kesp->SetN2kCANSendFrameBufSize(250);
  n2kesp->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  n2kesp->SetProductInformation("20240530", // Manufacturer's Model serial code
                                 111, // Manufacturer's product code
                                 "ESP32 Compass",  // Manufacturer's Model ID
                                 "1.0",  // Manufacturer's Software version code
                                 "1.0" // Manufacturer's Model version
                                 );
  // Set device information
  n2kesp->SetDeviceInformation(20240530, // Unique number. Use e.g. Serial number.
                                140, // Attitude (heading)
                                60, // Navigation
                                2046); 
  logToAll("Starting compass xmit...\n");

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  n2kesp->SetMode(tNMEA2000::N2km_NodeOnly);
  //n2kesp->SetMode(tNMEA2000::N2km_ListenAndNode);
  //n2kesp->SetForwardStream(&Serial);
  //n2kesp->SetForwardType(tNMEA2000::fwdt_Text);
  //n2kesp->EnableForward(true);
  //n2kesp->SetForwardOwnMessages(false);
  n2kesp->ExtendTransmitMessages(TransmitMessages);
  //n2kesp->SetOnOpen(OnN2kOpen);
  n2kesp->SetMsgHandler(HandleNMEA2000Msg);
  if (!n2kesp->Open())
    Serial.println("cannot open n2k");
  else Serial.println("n2k open");
}

tN2kMsg N2kMsg;

void SendN2kCompass(float heading) {
    //logToAll("sending compass on n2k: " + String(heading));
    SetN2kPGN127250(N2kMsg, 1, heading*DEGTORAD, N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
    //N2kMsg.Print(&Serial);
    if (n2kesp->SendMsg(N2kMsg))
      num_n2k_sent++;
    else logToAll("compass message send failed\n");
}

#if 0
void deleteCreateN2kReact() {
  n2kReact = app.onRepeat(compassParams.frequency, []() {
    heading = getCompassHeading(compassParams.variation, compassParams.orientation);
#ifdef N2K
    SendN2kCompass(heading);
#endif
  });  
}
#endif
