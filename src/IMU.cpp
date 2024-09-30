
#include "windparse.h"
#include "compass.h"
#ifdef HALLSENS
#include "SparkFun_TMAG5273_Arduino_Library.h"
#endif
#ifdef BNO08X
#include <Adafruit_BNO08x.h>
#endif

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8;
float comp16;
static int variation;
float mastCompassDeg, mastCompassMag; 
float accuracy;
int calStatus;
float boatCompassDeg, boatAccuracy;
float mastDelta;
extern bool teleplot;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
int mastOrientation=0, sensOrientation, boatOrientation;
extern int compassFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

extern bool compassOnToggle;

extern AsyncWebServer server;
extern AsyncEventSource events;

float getCompass(int correction);
void logToAll(String message);
void i2cScan(TwoWire Wire);

// for sending mast compass on bus
extern tNMEA2000 *n2kMain;
extern tN2kMsg correctN2kMsg;

#ifdef BNO08X
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
int reportType;
int IMUreports[SH2_MAX_SENSOR_ID];
int totalReports;
#endif

float calculateHeading(float r, float i, float j, float k, int correction);

#ifdef BNO08X
void setReports(int rType) {
  logToAll("Setting compass report to: 0x" + String(reportType,HEX));
    if (!bno08x.enableReport(reportType, compassParams.frequency*1000))
    logToAll("could not set report type: " + String(reportType,HEX));  
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_GRV, compassParams.frequency*10000))
      logToAll("could not set report type (2): " + String(SH2_ARVR_STABILIZED_GRV,HEX));
    else logToAll("enabled " + String(SH2_ARVR_STABILIZED_GRV,HEX));
}

float getCompassHeading(int correction) {
  if (bno08x.wasReset()) {
    logToAll("sensor was reset ");
    setReports(reportType);
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  /* Status of a sensor
   *   0 - Unreliable
   *   1 - Accuracy low
   *   2 - Accuracy medium
   *   3 - Accuracy high
   */
  float heading;

  IMUreports[sensorValue.sensorId]++;
  totalReports++;
  switch (sensorValue.sensorId) {
  case SH2_GAME_ROTATION_VECTOR:
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:
    accuracy = sensorValue.un.rotationVector.accuracy;
    calStatus = sensorValue.status;
    mastCompassMag = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, correction);      
    if (teleplot) {
      Serial.printf(">mag:%0.2f\n", mastCompassMag);
      Serial.printf(">acc:%0.2f\n", accuracy*RADTODEG);
      Serial.printf(">cal:%d\n", calStatus);
    }
    // NOTE we are returning mastCompassDeg NOT mastCompassMag
    // i.e. the last (relative) bearing we calculated
    return mastCompassDeg;
    break;
  case SH2_ARVR_STABILIZED_GRV:
    heading = calculateHeading(sensorValue.un.arvrStabilizedGRV.real, sensorValue.un.arvrStabilizedGRV.i, sensorValue.un.arvrStabilizedGRV.j, sensorValue.un.arvrStabilizedGRV.k, correction);      
    if (teleplot) Serial.printf(">IMU:%0.2f\n", heading);
    return heading;
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    logToAll("gyroscope calibrated");
    return heading;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    logToAll("magnetic field calibrated");
    return heading;
    break;
  default:
    logToAll("getBNO085() got unknown sensor id: 0x" + String(sensorValue.sensorId, HEX));
    return heading;
    break;
  }
  return -4.0;
}
#endif

// Function to calculate tilt-compensated heading from a quaternion
float calculateHeading(float r, float i, float j, float k, int correction) {
  //Serial.printf("r %0.2f i %0.2f j %0.2f k %0.2f %2d ", r, i, j, k, correction);
  // Convert quaternion to rotation matrix
  float r11 = 1 - 2 * (j * j + k * k);
  // change r21 to =-2 if sensor is upside down
  float r21 = 2 * (i * j + r * k);
  float r31 = 2 * (i * k - r * j);
  float r32 = 2 * (j * k + r * i);
  float r33 = 1 - 2 * (i * i + j * j);
  ////Serial.printf("r11 %0.2f r21 %0.2f r31 %0.2f r32 %0.2f r33 %0.2f ", r11, r21, r31, r32, r33);
  // Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);
  // Calculate yaw (psi)
  float psi = atan2(-r21, r11);
  float heading = (psi * RADTODEG) + (float)correction;
  ////Serial.printf("theta %0.2f phi %0.2f psi %0.2f h %0.2f ", theta, phi, psi, heading);
  ////Serial.printf("h %0.2f ", heading);
  // correction may be positive or negative
  if ((int)heading > 359) {
    heading -= 360.0;
  }
  if ((int)heading < 0) {
    heading += 360.0;
  }
  //Serial.printf("h %0.2f\n", heading);
  return heading;
}
