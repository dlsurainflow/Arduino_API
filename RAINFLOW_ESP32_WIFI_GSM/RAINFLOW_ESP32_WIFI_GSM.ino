// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define GPS_RX  35              // GSM/GPRS Module RX Pin
#define GPS_TX  34              // GSM/GPRS Module TX Pin
#define RGPIN 18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
//#define MODE_WIFI               // Use Wifi for Data Telemetry
#define MODE_GSM                // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE


#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif


#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <TaskScheduler.h>
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

//Network details
const char apn[]  = "smartlte";
const char user[] = "";
const char pass[] = "";

// RAFT Details
const char* APIKey  = "861db3ff0-9c48-43ab-91b8-346b94e498ad   ";   // Change to API-Key

#ifdef MODE_WIFI
WiFiClient client;
#endif

#ifdef MODE_GSM
HardwareSerial serialGSM(1);
TinyGsm modem(serialGSM);
TinyGsmClient client(modem);
#endif


TinyGPSPlus gps;

HardwareSerial SerialGPS(1);
RainFLOW rainflow;

void publishData();

//Task publishDataScheduler(300000, TASK_FOREVER, &publishData);
Task publishDataScheduler(10000, TASK_FOREVER, &publishData);
Scheduler runner;


#ifdef MODE_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char* apn, const char* gprsUser, const char* gprsPass) {
  serialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);       // Initialise serial connection to GSM module

  if (!modem.restart()) {                                     // Restarts GSM Modem
    DEBUG_PRINT("Failed to restart modem. Trying in 10s.");
    delay(3000);
    serialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
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
//  DEBUG_PRINT("IP Addr:        " + modem.localIP());
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
  Serial.println("Im Here");
  DEBUG_PRINT("Connected.");
  //DEBUG_PRINT("Connected. My IP Address is: " + WiFi.localIP());
  delay(1000);
}
#endif

void attachRainGauge(int rainGaugePin) {
  Serial.println("Attaching Raing Gauge to pin : " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
}

void tippingBucket() {
  //  This has not been calibrated yet as it will depend on your printing dimensions.
  //  As a rough guide, this rain gauge tips 15 times per 100 mL, over a radious of 5.72 cm
  //  One tip is ~ 6.67mL or 6.67cm^3.
  //  Catachment area = pi*r*r=102.79cm^2
  //  Rainfall per tip = 6.67mL/102.79 = 0.6489mL/tip

  int tipInterval = 100; // 100ms
  if (millis() - lastDetectedTipMillis > tipInterval) {
    rainfallAmount += tipAmount;
    DEBUG_PRINT("Bucket tipped! Total Rainfall: "  + String(rainfallAmount) + "mL");
    lastDetectedTipMillis = millis();
  }
}

void attachGPS() {
  Serial.println("Attaching GPS. RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
  SerialGPS.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  DEBUG_PRINT("Attaching GPS. RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
}

void setup() {
  Serial.begin(115200);
  Serial.println("System initialising");
  rainflow.rainflow(client);
  attachGPS();
  attachRainGauge(RGPIN);
  //connectWifi(ssid, password);
  connectGSM(9600, GSM_TX, GSM_RX, apn, user, pass);
  rainflow.connectServer(APIKey);
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();
}

void getData() {
  rainflow.addData("APIKey",      APIKey);
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      String Date = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
      rainflow.addData("Date",        Date);
      String Time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      rainflow.addData("Time",        Time);
      rainflow.addData("Latitude",    String(gps.location.lat()));
      rainflow.addData("Longitude",   String(gps.location.lng()));
      rainflow.addData("Altitude",    String(gps.altitude.meters()));
    }
  }
  rainflow.addData("floodDepth",  "0.00");
  rainflow.addData("rainfallAmt", String(rainfallAmount));
  rainflow.addData("battPercent", "N/A");
  rainflow.addData("battVoltage", "N.A");
}

void publishData() {
  getData();
  rainflow.publishData(APIKey);
}

void loop() {
  runner.execute();
}
