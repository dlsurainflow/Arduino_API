// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define GPS_RX  34              // GSM/GPRS Module RX Pin
#define GPS_TX  35              // GSM/GPRS Module TX Pin
#define RGPIN 18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define MODE_WIFI               // Use Wifi for Data Telemetry
#define DEBUG_MODE
//#define MODE_GSM                // Use GSM/GPRS for Data Telemetry

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif


#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include "rainflow.h"

//WiFi Details
const char* ssid                  = "myWiFi";
const char* password              = "accessingwifi";
const char* serverAddr            = "test.mosquitto.org";   //REMOVE ENTRY ONCE SERVER IS ESTABLISHED
const short unsigned serverPort   = 1883;
double rainfallAmount   = 0;          // Total Amount of Rainfall
double tipAmount        = 0.6489;     // Calibrated Tipping Amount
unsigned long lastDetectedTipMillis;

// RAFT Details
const char* APIKey  = "86ffc18d-8377";   // Change to API-Key

WiFiClient client;
TinyGPSPlus gps;
HardwareSerial SerialGSM(1);
RainFLOW rainflow;


#ifdef MODE_GSM


void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char* apn, const char* gprsUser, const char* gprsPass) {
  SerialAT.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);

  if (!modem.restart()) {
    DEBUG_PRINT("Failed to restart modem. Trying in 10s.");
    delay(3000);
    SerialAT.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    delay(10000);
    return;
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());

  //Connect to GPRS
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    DEBUG_PRINT("Connecting to " + String(apn));
    delay(10000);
  }

  if (modem.isGprsConnected()) {
    DEBUG_PRINT("Connected to " + String(apn));
  } else {
    DEBUG_PRINT("Disconnected!");
  }

  DEBUG_PRINT("IMEI:           " + String(modem.getIMEI()));
  DEBUG_PRINT("Operator:       " + String(modem.getOperator()));
  DEBUG_PRINT("IP Addr:        " + modem.localIP());
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));
  DEBUG_PRINT("GSM Time:       " + String(modem.getGSMDateTime(DATE_TIME)));
  DEBUG_PRINT("GSM Date:       " + String(modem.getGSMDateTime(DATE_DATE)));
}
#endif

#ifdef MODE_WIFI
void connectWifi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Attempting to connect..."        );
    delay(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }
  }
  DEBUG_PRINT("Connected. My IP Address is: " + WiFi.localIP());
}

#endif

void attachRainGauge(int rainGaugePin) {
  Serial.println("Attaching Raing Gauge to pin : " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
}

void tippingBucket() {
  //this has not been calibrated as it will depend on your own printing dimensions
  //as a rough guide though, this system has 15 tips per 100ml, over a radius of 5.72cm
  //one tip is approx 6.67mL, or 6.67cmCubed. Area of catchment= pi*5.72*5.62=102.79cm Squared
  //Rainfal per tip is 6.67mL/102.79=0.6489mL/tip
  int tipInterval = 100; // 100ms
  if (millis() - lastDetectedTipMillis > tipInterval) {
    rainfallAmount += tipAmount;
    DEBUG_PRINT("Bucket tipped! Total Rainfall: "  + String(rainfallAmount) + "mL");
    lastDetectedTipMillis = millis();
  }
}

void attachGPS() {
  SerialGSM.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  rainflow.connectServer(APIKey);
}

void setup(){
  Serial.begin(115200);
  Serial.println("System initialising");
  rainflow.rainflow(client);
  //attachGPS();
  attachRainGauge(RGPIN);
  connectWifi(ssid, password);
  rainflow.connectServer(APIKey);
}

void loop() {
  rainflow.addData("APIKey",      APIKey);
  while (SerialGSM.available() > 0) {
    if (gps.encode(SerialGSM.read())) {
      String Date = String(gps.date.month())+ "/" + String(gps.date.day()) + "/" + String(gps.date.year());
      rainflow.addData("Date",        Date);
      String Time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      rainflow.addData("Time",        Time);
      rainflow.addData("Latitude",    String(gps.location.lat()));
      rainflow.addData("Longitude",   String(gps.location.lng()));
      rainflow.addData("Altitude",    String(gps.altitude.meters()));
    }
  }
  rainflow.addData("floodDepth",  APIKey);
  rainflow.addData("rainfallAmt", String(rainfallAmount));
  rainflow.addData("battPercent", "N/A");
  rainflow.addData("battVoltage", "N.A");
  rainflow.publishData(APIKey);
    delay(10000);

  Serial.println("Test");
}
