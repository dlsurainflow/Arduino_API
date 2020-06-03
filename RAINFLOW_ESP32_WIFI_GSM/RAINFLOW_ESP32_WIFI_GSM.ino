//  -- RAFT DEFINTIONS
#define MICRO_BAUD_RATE 115200  // Microcontroller Serial Port Baud Rate
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define BATTMINVOLT 3.2         // Minimum Battery Voltage
#define BATTERYPIN 34           // Battery PIN
#define BATTERYRATIO 1          // Battery voltage divider ratio
//#define BATTERYRATIO 0.71825    // Battery voltage divider
#define LED 23                  // LED Indicator Pin
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds */
#define MODEM_WIFI              // Use Wifi for Data Telemetry
//#define MODEM_GSM               // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE              // DEBUG MODE ON
// -- GSM DEFINITIONS
#define TINY_GSM_MODEM_SIM800   // GSM/GPRS Module Model
#define GSM_RX    17             // GSM/GPRS Module RX Pin
#define GSM_TX    16             // GSM/GPRS Module TX Pin
#define GSM_BAUD  9600          // GSM/GPRS Module Baud Rate
// -- ULTRASONIC SENSOR DEFINITIONS [FOR FLOOD DEPTH]
#define US_RX 14                // Ultrasonic Module RX Pin
#define US_TX 12                // Ultrasonic Module TX 
#define US_MAXHEIGHT 600        // Ultrasonic Max Height (cm)
// -- GPS MODULE DEFINITIONS
#define GPS_RX  25              // GSM/GPRS Module RX Pin
#define GPS_TX  26              // GSM/GPRS Module TX Pin
#define GPS_BAUD  9600          // GSM/GPRS Module TX Pin
// -- RAIN GAUGE DEFINITIONS
#define RGPIN 35                // Rain Guage Pin
#define GPIO_NUM_X  GPIO_NUM_35 // 
#define tipAmount 0.6489
// -- BAROMETER DEFINITIONS
#define SEALEVELPRESSURE_HPA (1008.00)  // Change to mean sea level pressure in your area
#define BMP_SCK  (18)
#define BMP_MISO (19)
#define BMP_MOSI (23)
#define BMP_CS   (5)


#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Smoothed.h>
#include <NewPing.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Time.h>

#include <TimeLib.h>
#include "rainflow.h"
#ifdef MODEM_GSM
#include <TinyGsmClient.h>
#endif
#ifdef MODEM_WIFI
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

//WiFi Access Point Details
const char* ssid      = "Hidden Network";
const char* wifi_pass = "mmbmh15464";

//GSM Internet Details
const char* apn       = "smartlte";
const char* gprsUser  = "";
const char* gprsPass  = "";

// Ultrasonic Sensor Variables
RTC_DATA_ATTR float raftHeight = 0;                 // Set original Height
float floodDepth = 0;                               // Water level

// RAFT Details
const char* clientID  = "13130516a6f241e1b991f9293e9f29b99911";
const char* username  = "13130516a6f241e1b991f9293e9f29b99911";
const char* password  = "416dfaf311688498f4666ebdf34992f9eb61";
const char* streamID  = "RAFT_Data";


RTC_DATA_ATTR int bootCount = 0;
//int incomingByte = 0;
long currentmillis = 0;


//int test = 0;


#ifdef MODEM_WIFI
WiFiClient client;
#endif

#ifdef MODEM_GSM
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);
#endif

RainFLOW rainflow;
Adafruit_BMP280 bmp;

void publishData();
void modeCheck();

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
NewPing sonar(US_RX, US_TX, US_MAXHEIGHT);
//Task publishDataScheduler(60e3, TASK_FOREVER, &publishData);    // Every 1 minute 60seconds * 1000
//ask checkMode(180e4, TASK_FOREVER, &modeCheck);                // Every 30 minutes

Task publishDataScheduler(1, TASK_FOREVER, &publishData);
Task checkMode(2, TASK_FOREVER, &modeCheck);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
Smoothed<float> waterDepthSensor;
Scheduler Runner;

/////////////////////////  RAIN GAUGE  ///////////////////////
//  This has not been calibrated yet as it will depend on your printing dimensions.
//  As a rough guide, this rain gauge tips 15 times per 100 mL, over a radious of 5.72 cm
//  One tip is ~ 6.67mL or 6.67cm^3.
//  Catachment area = pi*r*r=102.79cm^2
//  Rainfall per tip = 6.67mL/102.79 = 0.6489mL/tip

RTC_DATA_ATTR int tipCount    = 0;                  // Total Amount of Tips

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR tippingBucket() {
  static unsigned long lastDetectedTipMillis;  // Time of Last Rain Guage Tip

  // Debounce for a quarter secon = max 4 counts/second
  if (millis() - lastDetectedTipMillis > 250)
  {
    portENTER_CRITICAL_ISR(&mux);
    tipCount++;

    DEBUG_PRINT("Tip Count:"  + String(tipCount));

    lastDetectedTipMillis = millis();
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void attachRainGauge(int rainGaugePin) {
  DEBUG_PRINT("Rain Gauge attached to pin: " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_X, 0);
}

float rainfallRate() {
  return tipAmount / 3600000;
}

float rainfallAmount() {
  return tipAmount * tipAmount;
}
/////////////////////////  END RAIN GAUGE  ///////////////////////

/////////////////////////  GSM/GPRS  ///////////////////////
#ifdef MODEM_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char* apn, const char* gprsUser, const char* gprsPass) {
  SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);       // Initialise serial connection to GSM module
  DEBUG_PRINT("Initialising GSM.");
  DEBUG_PRINT("BAUD: " + String(gsm_baud) + "\tTX" + String(gsm_tx) + "\tRX" + String(gsm_rx));
  DEBUG_PRINT("Initialising GSM.");
  while (!modem.restart()) {                                    // Restarts GSM Modem
    DEBUG_PRINT("Failed to restart modem. Trying in 5s.");
    wait(1000);
    SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    wait(4000);
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));

  //Connect to GPRS
  gprsConnect();
}

void gprsConnect() {
  DEBUG_PRINT("Connecting to GPRS.");
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    DEBUG_PRINT("Connecting to " + String(apn));
    //    delay(10000);
    wait(5000);
  }

  if (modem.isGprsConnected()) {
    DEBUG_PRINT("Connected to " + String(apn));
  } else {
    DEBUG_PRINT("Disconnected!");
  }

  //DEBUG_PRINT("IMEI:           " + String(modem.getIMEI()));
  DEBUG_PRINT("Operator:       " + String(modem.getOperator()));
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));
  //DEBUG_PRINT("GSM Date:       " + String(modem.getGSMDateTime(DATE_DATE)));
  //DEBUG_PRINT("GSM Time:       " + String(modem.getGSMDateTime(DATE_TIME)));
}

void sleepGSM() {
  DEBUG_PRINT("Sleeping GSM.");
  SerialGSM.println("AT+CSCLK=2");
}

void wakeupGSM() {
  DEBUG_PRINT("Waking Up GSM.");
  SerialGSM.println("AT");
  wait(500);
  SerialGSM.println("AT+CSCLK=0");
}

#endif
///////////////////////// END GSM/GPRS  ///////////////////////

///////////////////////// WIFI  ///////////////////////
#ifdef MODEM_WIFI
void connectWifi(const char* ssid, const char* password) {
  uint8_t i = 0;

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.disconnect(true);
  wait(1000);

  WiFi.mode(WIFI_STA);
  wait(1000);

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Attempting to connect..."        );
    wait(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }
  }

  wait(1000);
}

int32_t getRSSI(const char* target_ssid) {
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++) {
    if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0) {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}
#endif
///////////////////////// END WIFI  ///////////////////////

///////////////////////// GPS  ///////////////////////
void attachGPS() {
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_TX, GPS_RX);
  DEBUG_PRINT("Attached GPS @ RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
}

String getUnixTime() {
  tmElements_t te;
  time_t unixTime = 1590060818;
  smartDelay(5000);
  gps.encode(SerialGPS.read());
  while (!gps.time.isValid()) {
    gps.encode(SerialGPS.read());
    smartDelay(5000);
  }
  te.Second   = gps.time.second();
  te.Hour     = gps.time.hour();
  te.Minute   = gps.time.minute();
  te.Day      = gps.date.day();
  te.Month    = gps.date.month();
  te.Year     = gps.date.year() - (uint32_t)1970;
  unixTime    = makeTime(te);
  return (String)unixTime;
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

///////////////////////// END GPS  ///////////////////////

///////////////////////// ULTRASONIC  ///////////////////////
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
    wait(5);
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

///////////////////////// END ULTRASONIC  ///////////////////////

///////////////////////// BATTERY ///////////////////////
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

    wait(5);
  }
  double voltage = sum / sampleRate;
  return voltage;
}

double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  wait(5);
}
///////////////////////// END BATTERY ///////////////////////

///////////////////////// BAROMETRIC SENSOR ///////////////////////

void attachBarometer() {
  if (!bmp.begin(0x76)) {
    DEBUG_PRINT("Could not find a valid BMP280 sensor, check wiring!");
    return;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */
}

float getAltitude(){
  return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

float getPressure(){
  return (bmp.readPressure()/100.0F);
}

float getTemperature(){
  return bmp.readTemperature();
}

///////////////////////// END BAROMETRIC SENSOR ///////////////////////

void modeCheck() {
  //  float minFloodDepth = 10;
  //  unsigned long lastTipTime = 1.8e6;
  //  if ((getDepth() < minFloodDepth) && (lastDetectedTipMillis >= lastTipTime)) {
  //    DEBUG_PRINT("Mode: Standby");
  //    mode_Standby();
  //  }
  //  else if (((getDepth() >= minFloodDepth) && (lastDetectedTipMillis < lastTipTime)) && (getBatteryLevel() > BATTMINVOLT)) {
  //    DEBUG_PRINT("Mode: Continuous Monitoring");
  //    mode_ContinuousMonitoring();
  //  }
  //  else if (((getDepth() >= minFloodDepth) && (lastDetectedTipMillis < lastTipTime)) && (getBatteryLevel() <= BATTMINVOLT)) {
  //    DEBUG_PRINT("Mode: Battery Saver");
  //    mode_BatterySaver();
  //  }
  mode_Standby(); // Force to standby
  //mode_ContinuousMonitoring();
}

void mode_Standby() {
  DEBUG_PRINT("Standby Mode.");
  int minutesToSleep = 10;
  //publishDataScheduler.disable();
  publishData();
  ArduinoOTA.handle();
  sleep(minutesToSleep * 60);
  //modeCheck();
}

void mode_ContinuousMonitoring() {
  DEBUG_PRINT("Mode: Continuous Monitoring");
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
  DEBUG_PRINT("Mode: Battery Saver");
  int minutesToSleep = 10;
  publishDataScheduler.disable();

  publishData();
  sleep(minutesToSleep * 60);
  modeCheck();
}

void sleep(int time_to_sleep) {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 0);
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping for " + String(time_to_sleep) + " Seconds");
  rainflow.disconnect();
  Serial.println("Disconnected from server.");
  wait(100);
  Serial.flush();
  esp_deep_sleep_start();
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

void wait(unsigned long interval) {
  // In milliseconds (1s == 1000 ms)
  unsigned long time_now = millis();
  while (!(millis() - time_now >= interval)) {
  }
}

//String uptime() {
//  long days = 0, hours = 0, mins = 0, secs = 0;
//  secs = millis() / 1000;           // Convert milliseconds to seconds
//  mins = secs / 60;                 // Convert seconds to minutes
//  hours = mins / 60;                // Convert minutes to hours
//  days = hours / 24;                // Convert hours to days
//  secs = secs - (mins * 60);        // Subtract the coverted seconds to minutes in order to display 59 secs max
//  mins = mins - (hours * 60);       // Subtract the coverted minutes to hours in order to display 59 minutes max
//  hours = hours - (days * 24);      // Subtract the coverted hours to days in order to display 23 hours max
//
//  String uptimeBuffer = String(days) + ":" + String(hours) + ":" + String(mins) + ":" + String(secs);
//  return uptimeBuffer;
//}

void getData() {
  DEBUG_PRINT("Retrieving data.");
  String unixTime = getUnixTime();
  rainflow.addData("rainfallAmount", String(rainfallAmount()), unixTime);
  rainflow.addData("floodDepth", String(floodDepth), unixTime);
  rainflow.addData("Latitude", String(gps.location.lat()), unixTime);
  rainflow.addData("Longitude", String(gps.location.lng()), unixTime);
  rainflow.addData("Altitude", String(gps.altitude.meters()), unixTime);
  rainflow.addData("Altitude_1", String(getAltitude()), unixTime);
  rainflow.addData("Temperature", String(getTemperature()), unixTime);
  rainflow.addData("Pressure", String(getPressure()), unixTime);
  rainflow.addData("Satellites", String(gps.satellites.value()), unixTime);
  rainflow.addData("battVoltage", String(getBatteryVoltage()), unixTime);

#ifdef MODEM_WIFI
  rainflow.addData("RSSI_modem_2", String(getRSSI(ssid)), unixTime);
#endif
#ifdef MODEM_GSM
  rainflow.addData("RSSI_modem_2", String(modem.getSignalQuality()), unixTime);
#endif
}

void publishData() {
  getData();
#ifdef MODEM_WIFI
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi(ssid, wifi_pass);
  }
#endif
#ifdef MODEM_GSM
  gprsConnect();
#endif
  rainflow.publishData(clientID, username, password, streamID);
  fade();
}

void fade() {
  digitalWrite(LED_BUILTIN, HIGH);
  wait(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  wait(1000);                        // Delay is necessary as Serial takes some time
#endif
  DEBUG_PRINT("System initialising");
  setCpuFrequencyMhz(80);             // CPU Frequency set to 80MHz to reduce power consumption

  print_wakeup_reason();
  esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause();
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) tippingBucket();

  // -- Attach Sensors
  pinMode(BATTERYPIN, INPUT);         // Have to check if still necessary
  attachGPS();
  attachRainGauge(RGPIN);
  attachBarometer();
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef MODEM_WIFI
  connectWifi(ssid, wifi_pass);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG_PRINT("Start updating " + type);
  })

  .onEnd([]() {
    DEBUG_PRINT("\nEnd");
  })

  .onProgress([](unsigned int progress, unsigned int total) {
    //  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    DEBUG_PRINT("Progress: " + String(progress / (total / 100)));
  })

  .onError([](ota_error_t error) {
    //  Serial.printf("Error[%u]: ", error);
    DEBUG_PRINT("Error " + String(error) + " ");
    if (error == OTA_AUTH_ERROR) DEBUG_PRINT("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_PRINT("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_PRINT("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINT("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_PRINT("End Failed");
  });
  ArduinoOTA.begin();
#endif
#ifdef MODEM_GSM
  connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
#endif

  rainflow.rainflow(client);                            // Set RainFLOW API to use this client modem
  rainflow.connectServer(clientID, username, password); // Connects to RainFLOW Server

  Runner.init();
  Runner.addTask(publishDataScheduler);
  Runner.addTask(checkMode);
  checkMode.enable();
  modeCheck();
}

void loop() {
  Runner.execute();
  rainflow.keepAlive();
  ArduinoOTA.handle();
}
