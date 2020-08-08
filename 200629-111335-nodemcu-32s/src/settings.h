//  -- RAFT Settings
#define MICRO_BAUD_RATE 115200 // Microcontroller Serial Port Baud Rate
#define BATTMAXVOLT 4.2        // Maximum Battery Voltage
#define BATTMINVOLT 3.6        // Minimum Battery Voltage
#define BATTERYPIN 34          // Battery PIN
#define BATTERYRATIO 0.78      // Battery voltage divider ratio
#define LEDPin 18              // LED Indicator Pin
#define LEDPin1 19             // LED Indicator Pin
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds */
#define MODEM_WIFI             // Use Wifi for Data Telemetry
// #define MODEM_GSM  // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE // DEBUG MODE ON

//* -- GSM DEFINITIONS
#define TINY_GSM_MODEM_SIM800 // GSM/GPRS Module Model
#define GSM_BAUD 9600         // GSM/GPRS Module Baud Rate
#define GSM_RX 17             // GSM/GPRS Module RX Pin
#define GSM_TX 16             // GSM/GPRS Module TX Pin

//* -- ULTRASONIC SENSOR DEFINITIONS [FOR FLOOD DEPTH]
#define US_RX 14          // Ultrasonic Module RX Pin
#define US_TX 12          // Ultrasonic Module TX
#define US_MAXHEIGHT 600  // Ultrasonic Max Height (cm)
#define US_resetButton 13 // Reset Height Button

//* -- GPS MODULE DEFINITIONS
#define GPS_BAUD 9600 // GSM/GPRS Module TX Pin
#define GPS_RX 26     // GSM/GPRS Module RX Pin
#define GPS_TX 27     // GSM/GPRS Module TX Pin

//* -- RAIN GAUGE DEFINITIONS
#define rainGaugePin 35              // Rain Guage 1 Pin
#define rainGaugePin2 32             // Rain Gauge 2
#define GPIO_PIN_BITMASK 0x900004000 // (2^35 + 2^32)Hex
#define tipAmount 0.3636             // 0.3636 mm of rainfall per tip
#define tipAmount2 0.2555            // S

//* -- BAROMETER DEFINITIONS
#define SEALEVELPRESSURE_HPA (1013.25) // Change to mean sea level pressure in your area

#ifdef MODEM_GSM
#include <TinyGsmClient.h>
#endif
#ifdef MODEM_WIFI
#include <WiFi.h>
#endif

//* WiFi Access Point Details
const char *ssid = "Hidden Network";
const char *wifi_pass = "mmbmh15464";

//* GSM Internet Details
const char *apn = "smartlte";
const char *gprsUser = "";
const char *gprsPass = "";

//* RAFT Details
const char *clientID = "bbb1691accc836be0958909cf8426e22b246";
const char *username = "bbb1691accc836be0958909cf8426e22b246";
const char *password = "aeff9fb2b53b2eba1b2ca8b218514615f995";
const char *streamIDData = "RGAPI";
const char *streamIDInfo = "RAFT_Info";
