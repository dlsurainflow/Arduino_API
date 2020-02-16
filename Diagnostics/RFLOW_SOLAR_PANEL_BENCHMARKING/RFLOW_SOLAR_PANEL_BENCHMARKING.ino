// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define GPS_RX  35              // GSM/GPRS Module RX Pin
#define GPS_TX  34              // GSM/GPRS Module TX Pin
#define RGPIN 18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define MODE_WIFI               // Use Wifi for Data Telemetry
//#define MODE_GSM                // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE


#define SP1C_PIN 15
#define SP1V_PIN 16
#define SPRC_PIN 17
#define BV_PIN   18

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
#include <Smoothed.h>

//WiFi Details
const char* ssid                  = "myWiFi";
const char* password              = "accessingwifi";

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


//TinyGPSPlus gps;

//HardwareSerial SerialGPS(1);
RainFLOW rainflow;

void publishData();

Task publishDataScheduler(10000, TASK_FOREVER, &publishData);
Scheduler runner;

Smoothed<double> SP1C;  // Solar Panel 1 Current
Smoothed<double> SP1V;  // Solar panel 1 Voltage
Smoothed<double> SPRC;  // Solar Panel Reference Current
Smoothed<double> BV;    // Battery Voltage



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

void setup() {
  Serial.begin(115200);
  Serial.println("System initialising");
  rainflow.rainflow(client);
  //attachGPS();
  //attachRainGauge(RGPIN);
  connectWifi(ssid, password);
  //connectGSM(9600, GSM_TX, GSM_RX, apn, user, pass);
  rainflow.connectServer(APIKey);
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();
}

void getData() {
  double offsetVoltage = 2475, sensitivity = 185;

  double SP1Current = 0.0, SP1Voltage = 0.0, SPRCurrent = 0.0, BVoltage = 0.0;
  double SP1CVoltage = 0.0, SP1CValue = 0.0, SP1CSamples = 0.0, SP1CRaw = 0.0;
  double SP1VoltValue1 = 0.0, SP1VValue = 0.0, SP1VSamples = 0.0, SP1VRaw = 0.0;
  double SPRCVoltage = 0.0, SPRCValue = 0.0, SPRCSamples = 0.0, SPRCRaw = 0.0;
  double BValue1 = 0.0, BVValue = 0.0, BVSamples = 0.0, BVRaw = 0.0;
  
  // Solar Panel 1
  for (int x = 0; x < 150; x++) {
    float currentSensorValue1 = analogRead(SP1C_PIN);
    SP1C.add(currentSensorValue1);
    SP1CValue = SP1C.get();
    SP1CSamples = SP1CSamples + SP1CValue;

    float SP1VoltValue1 = analogRead(SP1V_PIN);
    SP1V.add(SP1VoltValue1);
    SP1VValue = SP1V.get();
    SP1VSamples = SP1VSamples + SP1VValue;

    float currentSensorValue2 = analogRead(SPRC_PIN);
    SPRC.add(currentSensorValue2);
    SPRCValue = SPRC.get();
    SPRCSamples = SPRCSamples + SPRCValue;

    float SPRCVoltage = analogRead(BV_PIN);
    BV.add(SP1VoltValue1);
    BVValue = BV.get();
    BVSamples = BVSamples + BVValue;

    delay (3);
  }

  SP1CRaw = (SP1CSamples / 150.0) + 100;
  SP1CVoltage = ((SP1CRaw / 4096.0)) * 3300;
  SP1Current = ((SP1CVoltage - offsetVoltage) / sensitivity) * 1000;

  SP1VRaw = (SP1VSamples / 150.0) + 100;
  SP1Voltage = ((SP1VRaw / 4096.0)) * 3300;
  double SP1Power = SP1Voltage * SP1Current;

  SPRCRaw = (SP1CSamples / 150.0) + 100;
  SPRCVoltage = ((SPRCRaw / 4096.0)) * 3300;
  SPRCurrent = ((SPRCVoltage - offsetVoltage) / sensitivity) * 1000;

  BVRaw = (BVSamples / 150.0) + 100;
  BVoltage = ((BVRaw / 4096.0)) * 3300;

  Serial.println("-------------------Solar Panel-------------------");
  Serial.println("Avg (ACS):        " + String(SP1CRaw));
  Serial.println("ACS Voltage:      " + String(SP1CVoltage));
  Serial.println("Current:          " + String(SP1Current));
  Serial.println("Avg (Volt):       " + String(SP1VRaw));
  Serial.println("Voltage:          " + String(SP1Voltage));
  Serial.println("Power:              " + String(SP1Power));
  Serial.println("-----------------Solar Panel (REF)-----------------");
  Serial.println("Avg (ACS):        " + String(SPRCRaw));
  Serial.println("ACS Voltage:      " + String(SPRCVoltage));
  Serial.println("Current:          " + String(SPRCurrent));
  Serial.println("----------------------Battery----------------------");
  Serial.println("Avg Voltage:        " + String(BVRaw));
  Serial.println("Voltage:           " + String(BVoltage));



  rainflow.addData("SP1 Raw",  String(SP1CRaw));
  rainflow.addData("SP1 ACS Voltage", String(SP1Voltage));
  rainflow.addData("SP1 Current",  String(SP1Current));
  rainflow.addData("SP1 RAW",  String(SP1CRaw));
  rainflow.addData("SP1 Voltage", String(SP1Voltage));
  rainflow.addData("SP1 Power", String(SP1Power));
  rainflow.addData("SPR Raw",  String(SPRCRaw));
  rainflow.addData("SPR ACS Voltage", String(SPRCVoltage));
  rainflow.addData("SPR Current",  String(SPRCurrent));
  rainflow.addData("BATT Raw", String(BVRaw));
  rainflow.addData("BATT Voltage",  String(BVoltage));
}

void publishData() {
  getData();
  rainflow.publishData(APIKey);
}

void loop() {
  runner.execute();
}
