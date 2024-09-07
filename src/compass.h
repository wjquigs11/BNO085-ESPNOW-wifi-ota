// compass control structure
typedef struct control_s {
  bool compassOnToggle = true;
  int orientation = 0;
  int variation = 0;
  int frequency = 100;
  char hostname[64] = "";
} control_s;
extern control_s compassParams;

#define DEGTORAD 0.01745

// moved to platformio.ini
//#define N2K // if N2K defined, init CAN bus and send N2K Heading PGN

#define BNOREADRATE 20 // msecs for 50Hz rate; optimum for BNO08x calibration

