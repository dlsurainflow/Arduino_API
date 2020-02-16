// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
#define US_RX 25                // Ultrasonic Module RX Pin
#define US_TX 26                // Ultrasonic Module TX 
#define US_MAXHEIGHT 400           // Ultrasonic Max Height (cm)
#define GPS_RX  34              // GSM/GPRS Module RX Pin
#define GPS_TX  35              // GSM/GPRS Module TX Pin
#define RGPIN 18                // Rain Guage Pin
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
//#define MODE_WIFI               // Use Wifi for Data Telemetry
#define DEBUG_MODE
#define MODE_GSM                // Use GSM/GPRS for Data Telemetry
#define Serial_GPS Serial1

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
#include <Time.h>
#include <NewPing.h>
#include <WiFi.h>
#include "rainflow.h"



//WiFi Details
const char* ssid                  = "myWiFi";
const char* password              = "accessingwifi";
const char* serverAddr            = "test.mosquitto.org";   //REMOVE ENTRY ONCE SERVER IS ESTABLISHED
const char* apn = "smartlte";
const char* gprsUser = "";
const char* gprsPass = "";
const short unsigned serverPort   = 1883;
double rainfallAmount   = 0;          // Total Amount of Rainfall
double tipAmount        = 0.6489;     // Calibrated Tipping Amount
double setHeight = 0;                 //Set original Height
double setDepth = 0;                  //Water level
unsigned long lastDetectedTipMillis;

int incomingByte =0;
long currentmillis=0;

// RAFT Details
const char* APIKey  = "48d619bb-6db4-45c4-af20-cd3f2719517e";  // Change to API-Key

//WiFiClient client;
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

TinyGPSPlus gps;

HardwareSerial SerialGPS(1);
RainFLOW rainflow;
NewPing sonar(US_RX, US_TX, US_MAXHEIGHT);

void publishData();

Task publishDataScheduler(300000, TASK_FOREVER, &publishData);
Scheduler runner;


#ifdef MODE_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char* apn, const char* gprsUser, const char* gprsPass) {
  SerialAT.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);       // Initialise serial connection to GSM module
  DEBUG_PRINT("Initialising GSM.");
  while (!modem.restart()) {                                    // Restarts GSM Modem
    DEBUG_PRINT("Failed to restart modem. Trying in 10s.");
    delay(1000);
    SerialAT.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    delay(4000);
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

void getGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      String Date = String(gps.date.month()) + " / " + String(gps.date.day()) + " / " + String(gps.date.year());
      rainflow.addData("Date",        Date);
      String Time = String(gps.time.hour()) + ": " + String(gps.time.minute()) + ": " + String(gps.time.second());
      rainflow.addData("Time",        Time);
      rainflow.addData("Latitude",    String(gps.location.lat()));
      rainflow.addData("Longitude",   String(gps.location.lng()));
      rainflow.addData("Altitude",    String(gps.altitude.meters()));
    }
  }
}

void getHeight() {
  setHeight = sonar.ping_cm();
  while (setHeight = 0.00) {
    setHeight = sonar.ping_cm();
  }
  Serial.print("Set Height: ");//to ground
  Serial.print(String(setHeight)); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("\nerror..");
  setHeight = sonar.ping_cm();
}

void getDepth() {
  setDepth = sonar.ping_cm();
  if (setDepth != 0.00) {
    Serial.println("Measured Distance: " + String(setDepth));
    setDepth = setHeight - setDepth;
    Serial.print("Depth: " + String(setDepth));
  } else {
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("System initialising");
  Serial.println("'Project Uptime");
  rainflow.rainflow(client);
  attachGPS();
  attachRainGauge(RGPIN);
  //connectWifi(ssid, password);
  connectGSM(9600, GSM_TX, GSM_RX, apn, gprsUser, gprsPass) ;
  rainflow.connectServer(APIKey);
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();
}

void getData() {
  rainflow.addData("APIKey",      APIKey);
  rainflow.addData("floodDepth",  String(setDepth));
  rainflow.addData("rainfallAmt", String(rainfallAmount));
  rainflow.addData("battPercent", "N / A");
  rainflow.addData("battVoltage", "N.A");
}   

void publishData() {
  getData();
  rainflow.publishData(APIKey);
}

void SerialCheck () {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    {
      if (incomingByte == 63) // if ? received then answer with data
      {
        currentmillis = millis(); // get the  current milliseconds from arduino
        // report milliseconds
        Serial.print("Total milliseconds running: ");
        Serial.println(currentmillis);
        uptime(); //call conversion function to display human readable time
      }
    }
  }
}
//######################################## SERIALCHECK ^^^^  #################################
//############################################ UPTIME vvvvv  #################################
void uptime()
{
  long days = 0;
  long hours = 0;
  long mins = 0;
  long secs = 0;
  secs = currentmillis / 1000; //convect milliseconds to seconds
  mins = secs / 60; //convert seconds to minutes
  hours = mins / 60; //convert minutes to hours
  days = hours / 24; //convert hours to days
  secs = secs - (mins * 60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24); //subtract the coverted hours to days in order to display 23 hours max
  //Display results
  Serial.println("Running Time");
  Serial.println("------------");
  if (days > 0) // days will displayed only if value is greater than zero
  {
    Serial.print(days);
    Serial.print(" days and :");
  }
  Serial.print(hours);
  Serial.print(":");
  Serial.print(mins);
  Serial.print(":");
  Serial.println(secs);
}

void loop() {
  runner.execute();
}
