// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Raten   
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define GPS_RX  35              // GSM/GPRS Module RX Pin
#define GPS_TX  34              // GSM/GPRS Module TX Pin
#define RGPIN   18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define MODE_WIFI               // Use Wifi for Data Telemetry
#define LED    5
#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

#include "rainflow.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

//WiFi Details
const char* ssid                  = "Hidden Network";
const char* password              = "mmbmh15464";


// RAFT Details
const char* APIKey  = "1429fedd-4956-4187-95aa";   // Change to API-Key
//const char* APIKey  = "861db3ff0-9c48-43ab-91b8";



WiFiClient client;
RainFLOW rainflow;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void publishData();

Task publishDataScheduler(10000, TASK_FOREVER, &publishData);
Scheduler runner;

String message;
DynamicJsonDocument doc(1024);

void printLocalTime() {
  String formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
}


void connectWifi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  // int a = esp_wifi_set_protocol(WIFI_STA, WIFI_PROTOCOL_LR);
  // Serial.println(a);
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    //DEBUG_PRINT("Attempting to connect..."        );
    delay(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }
  }
  DEBUG_PRINT("Connected.");
  DEBUG_PRINT("My IP Address: ");
  DEBUG_PRINT(WiFi.localIP());
  digitalWrite(LED, HIGH);
  delay(2500);
  digitalWrite(LED, LOW);
}

void getData() {
  timeClient.forceUpdate();
  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  String timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  rainflow.addData("Date",              dayStamp);
  rainflow.addData("Time",              timeStamp);
  rainflow.addData("SP1 Raw",           doc["SP1 Raw"]);
  rainflow.addData("SP1 ACS Voltage",   doc["SP1 ACS Voltage"]);
  rainflow.addData("SP1 Current",       doc["SP1 Current"]);
  rainflow.addData("SP1 VRaw",          doc["SP1 VRaw"]);
  rainflow.addData("SP1 Voltage",       doc["SP1 Voltage"]);
  rainflow.addData("SP1 Power",         doc["SP1 Power"]);
  rainflow.addData("SPR Raw",           doc["SPR Raw"]);
  rainflow.addData("SPR ACS Voltage",   doc["SPR ACS Voltage"]);
  rainflow.addData("SPR Current",       doc["SPR Current"]);
  rainflow.addData("BATT Raw",          doc["BATT Raw"]);
  rainflow.addData("BATT Voltage",      doc["BATT Voltage"]);
  doc.clear();
}

void publishData() {
  getData();
  rainflow.publishData(APIKey);
  digitalWrite(LED, HIGH);
  delay(25);
  digitalWrite(LED, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.println("System initialising");
  rainflow.rainflow(client);
  pinMode(LED, OUTPUT);
  connectWifi(ssid, password);
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  timeClient.forceUpdate();
  printLocalTime();
  rainflow.connectServer(APIKey);
}

void loop() {
  while (Serial.available()) {
    message = Serial.readString();
    Serial.println("Received message: ");
    Serial.println(message);
    DeserializationError error = deserializeJson(doc, message);
    if (error)
      return;
    timeClient.update();
    printLocalTime();
    publishData();
  }
  //runner.execute();
}
