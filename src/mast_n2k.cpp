// TBD: change params in n2k init so we're a "real" device

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <Wire.h>
//#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1305.h>
//#include <Fonts/FreeMono9pt7b.h>
//#include <Fonts/TomThumb.h>

#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25

// leaving here in case I decide to add a display later
// Used for software or hardware SPI
//#define OLED_CS 5
//#define OLED_DC 19

// Used for I2C or SPI
//#define OLED_RESET 22

// hardware SPI - use 7Mhz (7000000UL) or lower because the screen is rated for 4MHz, or it will remain blank!
//Adafruit_SSD1305 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS, 4000000UL);

tNMEA2000 *n2kesp;
extern int num_n2k_messages;

#define DEGTORAD 0.01745

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127250L,0};

void setupN2K() {
  n2kesp = new tNMEA2000_esp32(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN);
//tNMEA2000 &N2K=*(new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz,N2k_CAN_INT_PIN,MCP_CAN_RX_BUFFER_SIZE));

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
  Serial.println("Starting compass xmit...");
  //n2kesp->SetForwardStream(&Serial);
  //n2kesp->SetForwardType(tNMEA2000::fwdt_Text);
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  n2kesp->SetMode(tNMEA2000::N2km_NodeOnly,23);
  //n2kesp->EnableForward(true);
  //n2kesp->SetForwardOwnMessages(false);
  n2kesp->ExtendTransmitMessages(TransmitMessages);
  //n2kesp->SetOnOpen(OnN2kOpen);
  n2kesp->Open();
}

void SendN2kCompass(float heading) {
    tN2kMsg N2kMsg;
    tN2kHeadingReference ref = N2khr_magnetic; //N2khr_Unavailable;
    // sending with "unavailable" heading reference to (try to) avoid confusion
    //Serial.printf("sending compass %f on n2k\n", heading);
    SetN2kPGN127250(N2kMsg, 1, heading*DEGTORAD, 0, 0, ref);
    //N2kMsg.Print(&Serial);
    n2kesp->SendMsg(N2kMsg);
    num_n2k_messages++;
}

