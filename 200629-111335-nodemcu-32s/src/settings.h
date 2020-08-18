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

//* -- ULTRASONIC SENSOR SETTINGS [FOR FLOOD DEPTH]
#define US_RX 14          // Ultrasonic Module RX Pin
#define US_TX 12          // Ultrasonic Module TX
#define US_MAXHEIGHT 600  // Ultrasonic Max Height (cm)
#define US_resetButton 13 // Reset Height Button

//* -- GPS MODULE SETTINGS
#define GPS_BAUD 9600 // GSM/GPRS Module TX Pin
#define GPS_RX 26     // GSM/GPRS Module RX Pin
#define GPS_TX 27     // GSM/GPRS Module TX Pin

//* -- RAIN GAUGE SETTINGS
#define GPIO_PIN_BITMASK 0x800004000 // GPIO 35 (2^35 in Hex)
#define rainGaugePin 35              // Rain Guage Pin
#define tipAmount 0.3636             //  This has not been calibrated yet as it will depend on your printing dimensions.
/* 
To calculate the tipAmount (amount of rainfall per tip), calibrate the rain gauge by slowly putting 100mL of water into the rain gauge. Then
tipAmount = (100mL/#ofTips)/(pi*r*r)
Example:
Rain gauge tips 38 times with 100mL of water.
Rain gauge radius is 5.72 cm.
tipAmount  = (100mL/38tips)/(pi*5.72cm*5.72cm)
tipAmount = 0.0256cm/tip or 0.256mm 

NOTE: tipAmount should be in millimeters (mm)
*/

//* -- ULTRASONIC SENSOR SETTINGS
const int datasizeUS = 15;

//* -- BAROMETER SETTINGS
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea level pressure

//* Communications Protocol to Use
#ifdef MODEM_GSM
#include <TinyGsmClient.h>
#ifdef DEBUG_MODE
#define DUMP_AT_COMMANDS
#endif
#endif

#ifdef MODEM_WIFI
#include <WiFi.h>
#endif

//* WiFi Access Point Settings
const char *ssid = "Hidden Network";
const char *wifi_pass = "mmbmh15464";

//* GSM Internet Settings
const char *apn = "smartlte"; // For Smart Telecom
//const char *apn = "internet.globe.com.ph";  // For Globe Telecom
const char *gprsUser = "";
const char *gprsPass = "";
#define GATEWAY_NUMBER "+639XXXXXXXXX";

//* RAFT Credentials
const char *clientID = "bbb1691accc836be0958909cf8426e22b246";
const char *username = "bbb1691accc836be0958909cf8426e22b246";
const char *password = "aeff9fb2b53b2eba1b2ca8b218514615f995";
const char *streamIDData = "RGAPI";
const char *streamIDInfo = "RAFT_Info";
