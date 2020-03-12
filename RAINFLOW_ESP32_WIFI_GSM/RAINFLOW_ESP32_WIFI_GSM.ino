//  -- RAFT DEFINTIONS
#define MICRO_BAUD_RATE 115200  // Microcontroller Serial Port Baud Rate
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define BATTMINVOLT 3.2         // Minimum Battery Voltage
#define BATTERYPIN 33           // BATTERY PIN
#define BATTERYRATIO 0.71825
#define RGPIN 21                // Rain Guage Pin
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds */
//#define MODEM_WIFI              // Use Wifi for Data Telemetry
#define MODEM_GSM               // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE              // DEBUG MODE ON
// -- GSM DEFINITIONS
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX  17              // GSM/GPRS Module RX Pin
#define GSM_TX  16              // GSM/GPRS Module TX Pin
// -- ULTRASONIC SENSOR DEFINITIONS [FOR FLOOD DEPTH]
#define US_RX 25                // Ultrasonic Module RX Pin
#define US_TX 26                // Ultrasonic Module TX 
#define US_MAXHEIGHT 400        // Ultrasonic Max Height (cm)
// -- GPS MODULE DEFINITIONS
#define GPS_RX  34              // GSM/GPRS Module RX Pin
#define GPS_TX  35              // GSM/GPRS Module TX Pin
// #define Serial_GPS Serial1


#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <ArduinoJson.h>
//#include <TinyGPS++.h>
#include <Smoothed.h>
//#include <NewPing.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Time.h>
#include "rainflow.h"
#ifdef MODEM_GSM
#include <TinyGsmClient.h>
#endif
#ifdef MODEM_WIFI
#include <WiFi.h>
#endif

//WiFi Details
const char* ssid                  = "Hidden Network";
const char* password              = "mmbmh15464";

//GSM Details
const char* apn = "smartlte";
const char* gprsUser = "";
const char* gprsPass = "";

// Rain Gauge Variables
float rainfallAmount   = 0;          // Total Amount of Rainfall
float tipAmount        = 0.6489;     // Calibrated Tipping Amount
unsigned long lastDetectedTipMillis; // Time of Last Rain Guage Tip

// Ultrasonic Sensor Variables
float raftHeight = 0;                // Set original Height
//float floodDepth = 0;                // Water level

// RAFT Details
const char* APIKey  = "48d619bb-6db4-45c4";  // Change to API-Key


int incomingByte = 0;
long currentmillis = 0;
RTC_DATA_ATTR int bootCount = 0;

int test = 0;


#ifdef MODEM_WIFI
WiFiClient client;
#endif

#ifdef MODEM_GSM
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);
#endif

RainFLOW rainflow;

void publishData();
void modeCheck();

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
NewPing sonar(US_RX, US_TX, US_MAXHEIGHT);
//Task publishDataScheduler(60e3, TASK_FOREVER, &publishData);    // Every 1 minute 60seconds * 1000
//Task checkMode(180e4, TASK_FOREVER, &modeCheck);                // Every 30 minutes 

Task publishDataScheduler(60000, TASK_FOREVER, &publishData);
Task checkMode(30e4, TASK_FOREVER, &modeCheck);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
Smoothed<float> waterDepthSensor;
Scheduler Runner;

#ifdef MODEM_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char* apn, const char* gprsUser, const char* gprsPass) {
  SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);       // Initialise serial connection to GSM module
  DEBUG_PRINT("Initialising GSM.");
  DEBUG_PRINT("BAUD: " + String(gsm_baud) + "\tTX" + String(gsm_tx) + "\tRX" + String(gsm_rx));
  DEBUG_PRINT("Initialising GSM.");
  while (!modem.restart()) {                                    // Restarts GSM Modem
    DEBUG_PRINT("Failed to restart modem. Trying in 10s.");
    delay(1000);
    SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    delay(4000);
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());

  //Connect to GPRS
  gprsConnect();

}

void gprsConnect() {
  DEBUG_PRINT("Connecting to GPRS.");
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    DEBUG_PRINT("Connecting to " + String(apn));
    //    delay(10000);
    delay(5000);
  }

  if (modem.isGprsConnected()) {
    DEBUG_PRINT("Connected to " + String(apn));
  } else {
    DEBUG_PRINT("Disconnected!");
  }

  DEBUG_PRINT("IMEI:           " + String(modem.getIMEI()));
  DEBUG_PRINT("Operator:       " + String(modem.getOperator()));
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));
  DEBUG_PRINT("GSM Date:       " + String(modem.getGSMDateTime(DATE_DATE)));
  DEBUG_PRINT("GSM Time:       " + String(modem.getGSMDateTime(DATE_TIME)));
}

void sleepGSM() {
  DEBUG_PRINT("Sleeping GSM.");
  SerialGSM.println("AT+CSCLK=2");
}

void wakeupGSM() {
  DEBUG_PRINT("Waking Up GSM.");
  SerialGSM.println("AT");
  delay(500);
  SerialGSM.println("AT+CSCLK=0");
}
#endif

#ifdef MODEM_WIFI
void connectWifi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  wifiConnect();
  //DEBUG_PRINT("Connected. My IP Address is: " + WiFi.localIP());
  delay(1000);
}

void wifiConnect() {
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Attempting to connect..."        );
    delay(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }
  }
}

void printLocalTime() {
  String formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
}
#endif

void modeCheck() {
  float minFloodDepth = 10;
  unsigned long lastTipTime = 1.8e6;
    if ((getDepth() < minFloodDepth) && (lastDetectedTipMillis >= lastTipTime)) {
      DEBUG_PRINT("Mode: Standby");
      mode_Standby();
    }
    else if (((getDepth() >= minFloodDepth) && (lastDetectedTipMillis < lastTipTime)) && (getBatteryLevel() > BATTMINVOLT)) {
      DEBUG_PRINT("Mode: Continuous Monitoring");
      mode_ContinuousMonitoring();
    }
    else if (((getDepth() >= minFloodDepth) && (lastDetectedTipMillis < lastTipTime)) && (getBatteryLevel() <= BATTMINVOLT)) {
      DEBUG_PRINT("Mode: Battery Saver");
      mode_BatterySaver();
    }
  // mode_Standby(); // Force to standby 
}

void mode_Standby() {
  DEBUG_PRINT("Standby Mode.");
  int minutesToSleep = 1;
  publishDataScheduler.disable();
  publishData();
  sleep(minutesToSleep * 60);
  modeCheck();
}

void mode_ContinuousMonitoring() {
  DEBUG_PRINT("Continous Monitoring Mode.");
  if (!publishDataScheduler.isEnabled()) {
    publishDataScheduler.enable();
  }
  if (!checkMode.isEnabled()) {
    checkMode.enable();
  }
  fade();
  loop();
}

void mode_BatterySaver() {
  DEBUG_PRINT("Battery Saver Mode.");
  int minutesToSleep = 2;
  publishDataScheduler.disable();

  publishData();
  sleep(minutesToSleep * 60);
  modeCheck();
}

void sleep(int time_to_sleep) {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 0);
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping for " + String(time_to_sleep) + " Seconds");

  delay(100);
  Serial.flush();
  esp_light_sleep_start();
 
  print_wakeup_reason();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : DEBUG_PRINT("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1  : DEBUG_PRINT("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : DEBUG_PRINT("Wakeup caused by timer"); break;
    default :
      DEBUG_PRINT("Wakeup was not caused by deep sleep:");
      DEBUG_PRINT(String(wakeup_reason));
      break;
  }
}

void attachRainGauge(int rainGaugePin) {
  DEBUG_PRINT("Rain Gauge attached to pin: " + String(rainGaugePin));
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
  SerialGPS.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  DEBUG_PRINT("Attached GPS @ RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
}

void getGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      String Date = String(gps.date.month()) + " / " + String(gps.date.day()) + " / " + String(gps.date.year());
      String Time = String(gps.time.hour()) + ": " + String(gps.time.minute()) + ": " + String(gps.time.second());

      rainflow.addData("Date",        Date);
      rainflow.addData("Time",        Time);
      rainflow.addData("Latitude",    String(gps.location.lat()));
      rainflow.addData("Longitude",   String(gps.location.lng()));
      rainflow.addData("Altitude",    String(gps.altitude.meters()));
    }
  }
}

void getHeight() {
  float sample = 0.00, raw = 0.00;
  int sampleRate = 50;

  for (int i = 0; i < sampleRate;) {
    float sensorValue = sonar.ping_cm();
    if (sensorValue != 0.00) {
      waterDepthSensor.add(sensorValue);
      sample = waterDepthSensor.get();
      raw = raw + sample;
      i++;
    }
    delay(5);
  }
  raftHeight = raw / sampleRate;
  DEBUG_PRINT("RAFT Height: " + String(raftHeight));
  waterDepthSensor.clear();
}

float getDepth() {
  float sensorValue, raw = 0.00, sample = 0.00, avg = 0.00;
  int sampleRate = 50;
  for (int i = 0; i < sampleRate;) {
    sensorValue = sonar.ping_cm();
    if (sensorValue != 0.00) {
      waterDepthSensor.add(sensorValue);
      sample = waterDepthSensor.get();
      raw = raw + sample;
      i++;
    }
  }
  avg = raw / sampleRate;
  float floodDepth = raftHeight - avg;
  DEBUG_PRINT("Flood Depth: " + String(floodDepth));
  waterDepthSensor.clear();

  return floodDepth;
}

float getBatteryLevel() {
  double voltage = getBatteryVoltage();
  if (voltage <= BATTMINVOLT) {
    return 0.00;
  }
  else if (voltage > BATTMAXVOLT) {
    return 100.00;
  }
  else {
    float batteryLevel = ((getBatteryVoltage() - BATTMINVOLT) / (BATTMAXVOLT - BATTMINVOLT)) * 100.00;
    return batteryLevel;
  }
}

double getBatteryVoltage() {
  int sampleRate = 50;
  double sum = 0;
  for (int j = 0; j < sampleRate; j++) {
    sum += ReadVoltage(BATTERYPIN) / BATTERYRATIO;
    
    delay(5);
  }
  double voltage = sum / sampleRate;
  return voltage;
}

double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  delay(5);
}

String uptime() {
  long days = 0, hours = 0, mins = 0, secs = 0;
  //secs = currentmillis / 1000;    // Convert milliseconds to seconds
  secs = millis() / 1000;           // Convert milliseconds to seconds
  mins = secs / 60;                 // Convert seconds to minutes
  hours = mins / 60;                // Convert minutes to hours
  days = hours / 24;                // Convert hours to days
  secs = secs - (mins * 60);        // Subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60);       // Subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24);      // Subtract the coverted hours to days in order to display 23 hours max

  String uptimeBuffer = String(days) + ":" + String(hours) + ":" + String(mins) + ":" + String(secs);
  return uptimeBuffer;
}

void getData() {
#ifdef MODEM_WIFI
  timeClient.forceUpdate();
  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  String timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  rainflow.addData("Date",              dayStamp);
  rainflow.addData("Time",              timeStamp);
#endif
#ifdef MODEM_GSM
  rainflow.addData("Date",              String(modem.getGSMDateTime(DATE_DATE)));
  rainflow.addData("Time",              String(modem.getGSMDateTime(DATE_TIME)));
#endif
  //rainflow.addData("floodDepth",  String(getDepth()));
  rainflow.addData("rainfallAmt", String(rainfallAmount));
  rainflow.addData("battLevel",   String(getBatteryLevel()));
  rainflow.addData("battVoltage", String(getBatteryVoltage()));
  rainflow.addData("rawBattVolt", String(ReadVoltage(BATTERYPIN)));
  rainflow.addData("Uptime",      uptime());
}

void publishData() {
  getData();
#ifdef MODEM_WIFI
  connectWifi(ssid, password);
#endif
#ifdef MODEM_GSM
  gprsConnect();
#endif
  rainflow.publishData(APIKey);
  fade();
}


void fade() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  DEBUG_PRINT("System initialising");

  //  pinMode(BATTERYPIN, INPUT);
  //  attachGPS();
  attachRainGauge(RGPIN);
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef MODEM_WIFI
  connectWifi(ssid, password);
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  timeClient.forceUpdate();
  printLocalTime();
#endif
#ifdef MODEM_GSM
  connectGSM(9600, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
#endif
  rainflow.rainflow(client);
  rainflow.connectServer(APIKey);
  Runner.init();
  Runner.addTask(publishDataScheduler);
  Runner.addTask(checkMode);
  checkMode.enable();
  modeCheck();
}

void loop() {
  Runner.execute();
}
