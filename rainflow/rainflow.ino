// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define RGPIN 18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage

// Libraries
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <HardwareSerial.h>

// Network Details
const char apn[]      = "smartlte";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server Details
const char* serverAddr            = "test.mosquitto.org";
const short unsigned serverPort   = 1883;


// Node Details
const char* APIKey  = "86ffc18d-8377-4653-bddc-df61fe88c448";   // Change to API-Key

// Variables
double rainfall   = 0;          // Total Amount of Rainfall
double tipAmount  = 0.6489;     // Calibrated Tipping Amount
float batteryPercent;
float batteryVoltage;
unsigned long lastDetectedTipMillis;
DynamicJsonDocument payloadData(1024);
DynamicJsonDocument payloadDevice(1024);

// Initialisation
HardwareSerial SerialAT(1);       // Serial Connection to GSM Module
TinyGsm modem(SerialAT);          // Instance Creation of TinyGSM
TinyGsmClient client(modem);      // Instance Creation of TinyGSM Client
PubSubClient rainflow(client);    // Instance Creation of MQTT

void setup() {
  Serial.begin(MICRO_BAUD_RATE);
  Serial.println("Initialising System...");
  attachRainGauge(RGPIN);
  connectGSM();
  getBatteryStatus();
  Serial.println("Battery Charge Percent: " + String(batteryPercent));
  Serial.println("Battery Voltage:        " + String(batteryVoltage));
  connectServer();
}

void connectGSM() {
  SerialAT.begin(9600, SERIAL_8N1, GSM_TX, GSM_RX);
  if (!modem.restart()) {
    Serial.println("Failed to restart modem. Trying in 10s.");
    delay(3000);
    SerialAT.begin(9600, SERIAL_8N1, GSM_TX, GSM_RX);
    delay(10000);
    return;
  }
  Serial.println("Modem Name: " + modem.getModemName());
  Serial.println("Modem Info: " + modem.getModemInfo());
  Serial.println("");
  //Connect to GPRS
  Serial.println("Connecting to " + String(apn));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    delay(10000);
    return;
  }
  if (modem.isGprsConnected()) {
    Serial.println("Connected");
  } else {
    Serial.println("Disconnected!");
  }
  Serial.println("IMEI:           " + String(modem.getIMEI()));
  Serial.println("Operator:       " + String(modem.getOperator()));
  Serial.println("IP Addr:        " + String(modem.localIP()));
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));
  Serial.println("GSM Time:       " + String(modem.getGSMDateTime(DATE_TIME)));
  Serial.println("GSM Date:       " + String(modem.getGSMDateTime(DATE_DATE)));
}

void connectServer() {
  rainflow.setServer(serverAddr, serverPort);
  rainflow.setCallback(rainflowCallback);
  Serial.println("Connecting to RainFLOW Server at " + String(serverAddr));
  while (connectMqtt() == false) continue;
  Serial.println();
}

void getBatteryStatus() {
  batteryVoltage = modem.getBattVoltage() / 1000.0;
  batteryPercent = batteryVoltage / BATTMAXVOLT * 100;
  Serial.println("Battery Charge Percent: " + String(batteryPercent));
  Serial.println("Battery Voltage:        " + String(batteryVoltage));
  Serial.println("");
}

bool connectMqtt() {
  if (!rainflow.connect("APIKey")) {
    Serial.print(".");
    return false;
  }
  Serial.println("Connected to server!");
  char topic[45];
  strcpy(topic, "status/");
  strcat(topic, APIKey);
  rainflow.subscribe(topic);
  return rainflow.connected();
}

void rainflowCallback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Received Message: ");
  Serial.write(payload, len);
  Serial.println();
}

void attachRainGauge(int rainGaugePin) {
  Serial.println("Attaching pin : " + String(rainGaugePin));
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
    rainfall += tipAmount;
    Serial.print("Bucket tipped! Total Rainfall: ");
    Serial.print(rainfall);
    Serial.println("mL");
    lastDetectedTipMillis = millis();
  }
}

void loop() {
  getBatteryStatus();
  String payloadBuffer;
  String topicBuffer;
  payloadData["API_Key"]      = APIKey;
  payloadData["Date"]         = String(modem.getGSMDateTime(DATE_DATE));
  payloadData["Time"]         = String(modem.getGSMDateTime(DATE_TIME));
  payloadData["Latitude"]     = "14.5647";
  payloadData["Longtitude"]   = "120.9932";
  payloadData["Altitude"]     = "64";
  payloadData["floodDepth"]   = "0";
  payloadData["rainfallAmt"]  = rainfall;
  serializeJson(payloadData, payloadBuffer);
  topicBuffer                 = "data/";
  topicBuffer                 += APIKey;
  rainflow.publish(topicBuffer.c_str(), payloadBuffer.c_str());
  
  payloadDevice["API_Key"]      = APIKey;
  payloadDevice["Date"]         = String(modem.getGSMDateTime(DATE_DATE));
  payloadDevice["Time"]         = String(modem.getGSMDateTime(DATE_TIME));
  payloadDevice["BattPercent"]  = batteryPercent;
  payloadDevice["BattVoltage"]  = batteryVoltage;
  serializeJson(payloadDevice, payloadBuffer);
  topicBuffer                   = "device/";
  topicBuffer                   += APIKey;
  rainflow.publish(topicBuffer.c_str(), payloadBuffer.c_str());

  delay(5000);

}
