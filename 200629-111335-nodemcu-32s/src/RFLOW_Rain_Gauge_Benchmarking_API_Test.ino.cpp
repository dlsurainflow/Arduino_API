# 1 "C:\\Users\\garan\\AppData\\Local\\Temp\\tmps072iivt"
#include <Arduino.h>
# 1 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
#include "settings.h"
#include <Arduino.h>

#include "BME280/BME280I2C.h"
#include "BME280/EnvironmentCalculations.h"
#include <AsyncMqttClient.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <Wire.h>
#include <SPI.h>
#include <MedianFilterLib.h>
#include "EEPROM.h"
#include <Update.h>
#include <WiFi.h>
#include <Wire.h>

#define FIRMWARE_VER 1.0




RTC_DATA_ATTR float raftHeight = 0;





float medianGetHeight = 0;
float medianHeight = 0;
float medianDepth = 0;
long lastDetectedTipMillisUS = 0;
bool heightWrite = false;

const int datasizeUS = 15;
MedianFilter<int> medianFilter(3);

RTC_DATA_ATTR int bootCount = 0;

long currentmillis = 0;

RTC_DATA_ATTR int currentMode = 0;

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) 
#endif

WiFiClient client;
AsyncMqttClient rainflowMQTT;

BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_16,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76);
BME280I2C bme(settings);

void publishData();
void modeCheck();
void rainfallAmountReset();

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);




Task publishDataScheduler(19 * 60e3, TASK_FOREVER, &publishData);
Task checkMode(5 * 60e3, TASK_FOREVER, &modeCheck);
void IRAM_ATTR tippingBucket();
void IRAM_ATTR tippingBucket2();
void attachRainGauge();
double rainfallRate();
double rainfallAmount();
float rainfallRate2();
float rainfallAmount2();
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char *apn, const char *gprsUser, const char *gprsPass);
void gprsConnect();
void sleepGSM();
void wakeupGSM();
void connectWifi(const char *ssid, const char *password);
void disconnectWifi();
int32_t getRSSI(const char *target_ssid);
void attachGPS();
String getUnixTime();
void sendPacket(byte *packet, byte len);
void GPS_powerSaveMode();
void GPS_maxPerformanceMode();
static void smartDelay(unsigned long ms);
void attachUS();
void getHeight();
void usISR();
void setHeight();
float getDepth();
float getBatteryLevel();
double getBatteryVoltage();
double ReadVoltage(byte pin);
void attachBarometer();
float getAltitude();
float getPressure();
float getTemperature();
float getHumidity();
float getDewPoint();
float getHeadIndex();
String getHeaderValue(String header, String headerName);
String getBinName(String url);
String getHostName(String url);
void update(char *payload);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);
void mode_Standby();
void mode_ContinuousMonitoring();
void mode_BatterySaver();
void sleep(int time_to_sleep);
void print_wakeup_reason();
void wait(unsigned long interval);
void dataPublish();
void infoPublish();
void indicatorLED(bool status);
void indicatorLED1(bool status);
void setup();
void loop();
#line 80 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
Task rainfallReset(10 * 60e3, TASK_FOREVER, &rainfallAmountReset);

Task setHeightTask(10, TASK_ONCE, &setHeight);
WiFiUDP ntpUDP;
Scheduler Runner;
# 93 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
RTC_DATA_ATTR int tipCount = 0, tipCount2 = 0;
RTC_DATA_ATTR int rainGaugeDate = 0;
static unsigned long lastDetectedTipMillis = 0, tipTime = 0, lastDetectedTipMillis2 = 0, tipTime2 = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR tippingBucket()
{


  if (millis() - lastDetectedTipMillis > 250)
  {
    portENTER_CRITICAL_ISR(&mux);

    tipTime = millis() - lastDetectedTipMillis;
    tipCount++;

    DEBUG_PRINT("Tip Count:" + String(tipCount));

    lastDetectedTipMillis = millis();



    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR tippingBucket2()
{



  if (millis() - lastDetectedTipMillis2 > 250)
  {
    tipTime2 = millis() - lastDetectedTipMillis2;
    portENTER_CRITICAL_ISR(&mux);
    tipCount2++;

    DEBUG_PRINT("Tip Count:" + String(tipCount2));

    lastDetectedTipMillis2 = millis();
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void attachRainGauge()
{
  DEBUG_PRINT("Rain Gauge 1 attached to pin: " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);


  DEBUG_PRINT("Rain Gauge 2 attached to pin: " + String(rainGaugePin2));
  pinMode(rainGaugePin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin2), tippingBucket2, FALLING);
}

void rainfallAmountReset()
{
  smartDelay(5000);
  if (gps.date.isValid())
  {
    if (rainGaugeDate != gps.date.day())
    {
      DEBUG_PRINT("Resetting rain gauge paramaters.");
      tipCount = 0;
      tipTime = 0;
      lastDetectedTipMillis = 0;
      tipCount2 = 0;
      tipTime2 = 0;
      lastDetectedTipMillis2 = 0;
      rainGaugeDate = gps.date.day();
    }
  }
  if ((millis() - lastDetectedTipMillis) >= 3.6e6)
  {
    lastDetectedTipMillis = 0;
    tipTime = 0;
  }
}

double rainfallRate()
{
  if (tipCount == 0 || tipTime == 0)
    return 0;
  else
  {
    return (double)((double)tipAmount * (double)3.6e6 / (double)(tipTime / 1.00));
  }
}

double rainfallAmount()
{
  return (double)((double)tipCount * tipAmount);
}

float rainfallRate2()
{
  if (tipCount2 == 0 || tipTime2 == 0)
    return 0;
  else
  {
    return (double)((double)tipAmount2 * (double)3.6e6 / (double)(tipTime2 / 1.00));
  }
}

float rainfallAmount2()
{
  return (double)((double)tipCount2 * tipAmount2);
}



#ifdef MODEM_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char *apn, const char *gprsUser, const char *gprsPass)
{
  SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
  DEBUG_PRINT("Initialising GSM.");
  DEBUG_PRINT("BAUD: " + String(gsm_baud) + "\tTX" + String(gsm_tx) + "\tRX" + String(gsm_rx));
  DEBUG_PRINT("Initialising GSM.");
  while (!modem.restart())
  {
    DEBUG_PRINT("Failed to restart modem. Trying in 5s.");
    wait(1000);
    SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    wait(4000);
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));


  gprsConnect();
}

void gprsConnect()
{
  DEBUG_PRINT("Connecting to GPRS.");
  while (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    DEBUG_PRINT("Connecting to " + String(apn));

    wait(5000);
  }

  if (modem.isGprsConnected())
  {
    DEBUG_PRINT("Connected to " + String(apn));
  }
  else
  {
    DEBUG_PRINT("Disconnected!");
  }


  DEBUG_PRINT("Operator:       " + String(modem.getOperator()));
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));


}

void sleepGSM()
{
  DEBUG_PRINT("Sleeping GSM.");
  SerialGSM.println("AT+CSCLK=2");
}

void wakeupGSM()
{
  DEBUG_PRINT("Waking Up GSM.");
  SerialGSM.println("AT");
  wait(500);
  SerialGSM.println("AT+CSCLK=0");
}

#endif



#ifdef MODEM_WIFI
void connectWifi(const char *ssid, const char *password)
{
  uint8_t i = 0;

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.disconnect(true);
  wait(1000);

  WiFi.mode(WIFI_STA);
  wait(2000);

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  unsigned long start = millis();
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - start) >= 30000))
  {
    DEBUG_PRINT("Attempting to connect...");
    wait(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }

    if ((millis() - start) >= 30000)
      break;
  }

  wait(5000);
}

void disconnectWifi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

int32_t getRSSI(const char *target_ssid)
{
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++)
  {
    if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0)
    {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}
#endif



void attachGPS()
{
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_TX, GPS_RX);
  DEBUG_PRINT("Attached GPS @ RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
}

String getUnixTime()
{
  tmElements_t te;
  time_t unixTime = 1590060818;
  smartDelay(5000);
  gps.encode(SerialGPS.read());
  while (!gps.time.isValid())
  {
    gps.encode(SerialGPS.read());
    smartDelay(5000);
  }
  te.Second = gps.time.second();
  te.Hour = gps.time.hour();
  te.Minute = gps.time.minute();
  te.Day = gps.date.day();
  te.Month = gps.date.month();
  te.Year = gps.date.year() - (uint32_t)1970;
  unixTime = makeTime(te);
  return (String)unixTime;
}

void sendPacket(byte *packet, byte len)
{
  for (byte i = 0; i < len; i++)
  {
    SerialGPS.write(packet[i]);
  }
}

void GPS_powerSaveMode()
{
  DEBUG_PRINT("Setting GPS to Power Save Mode (PSM).");
  byte packet[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
  sendPacket(packet, sizeof(packet));
}

void GPS_maxPerformanceMode()
{
  DEBUG_PRINT("Setting GPS to Power Save Mode (PSM).");
  byte packet[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
  sendPacket(packet, sizeof(packet));
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  }
}




void attachUS()
{
  DEBUG_PRINT("Attaching Ultrasonic Sensor. TX:" + String(US_TX) + " RX:" + String(US_RX) + " USReset:" + String(US_resetButton));
  pinMode(US_RX, OUTPUT);
  pinMode(US_TX, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_resetButton), usISR, FALLING);
}

void getHeight()
{
  float duration = 0;
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);

  for (int i = 0; i < datasizeUS; i++)
  {

    float temp = getTemperature();
    float hum = getHumidity();





    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH);




    raftHeight = (duration / 2) * ((331.4 + (0.606 * temp) + (0.0124 * hum)) / 10000);


    medianGetHeight = medianFilter.AddValue(raftHeight);





    DEBUG_PRINT("getHeight: " + String(i) + " - " + String(medianGetHeight));
    wait(1000);
  }

  indicatorLED(false);
}

void usISR()
{
  if (millis() - lastDetectedTipMillisUS > 10000)
  {
    DEBUG_PRINT("Height write enabled.");
    lastDetectedTipMillisUS = millis();
    heightWrite = true;
  }
}

void setHeight()
{
  DEBUG_PRINT("Saving height to memory.");
  EEPROM.writeFloat(0, medianGetHeight);
  EEPROM.commit();
  delay(50);
  setHeightTask.disable();
  DEBUG_PRINT("Succesfully saved height of " + String(medianGetHeight) + " in memory.");
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
}

float getDepth()
{
  float duration = 0, floodDepth = 0;

  medianHeight = EEPROM.readFloat(0);
  DEBUG_PRINT("Median Height: " + String(medianHeight));

  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);

  int i = 0, counter = 0;
  for (; i < datasizeUS;)
  {
    float temp = getTemperature();
    float hum = getHumidity();



    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH, 26000);





    floodDepth = (duration / 2) * ((331.4 + (0.606 * temp) + (0.0124 * hum)) / 10000);


    if (abs(floodDepth) <= US_MAXHEIGHT)
    {
      medianDepth = medianFilter.AddValue(floodDepth);
      i++;
    }






    medianDepth = medianHeight - medianDepth;
    DEBUG_PRINT("getDepth: " + String(i) + " - " + String(medianDepth));
    counter++;

    if (counter >= (datasizeUS + 10))
      break;

    wait(1000);
  }
  floodDepth = medianDepth;
  indicatorLED(false);

  DEBUG_PRINT("Flood Depth: " + String(medianDepth));
  if (floodDepth < 0)
    return 0;
  else
    return floodDepth;
}




float getBatteryLevel()
{
  double voltage = getBatteryVoltage();
  if (voltage <= BATTMINVOLT)
  {
    return 0.00;
  }
  else if (voltage > BATTMAXVOLT)
  {
    return 100.00;
  }
  else
  {
    float batteryLevel = (voltage - BATTMINVOLT) / (BATTMAXVOLT - BATTMINVOLT) * 100.00;
    return batteryLevel;
  }
}

double getBatteryVoltage()
{
  int sampleRate = 50;
  double sum = 0;
  for (int j = 0; j < sampleRate; j++)
  {
    sum += ReadVoltage(BATTERYPIN) / BATTERYRATIO;

    wait(5);
  }
  double voltage = sum / sampleRate;
  return voltage;
}

double ReadVoltage(byte pin)
{
  double reading = analogRead(pin);
  if (reading < 1 || reading > 4095)
    return 0;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  wait(5);
}




void attachBarometer()
{
  DEBUG_PRINT("Attaching Barometer.");
  Wire.begin();
  unsigned long status = millis();
  while ((millis() - status < 5000) && !bme.begin())
  {
    DEBUG_PRINT("Connecting Barometer.");
  }

  switch (bme.chipModel())
  {
  case BME280::ChipModel_BME280:
    DEBUG_PRINT("Found BME280 sensor!");
    break;
  case BME280::ChipModel_BMP280:
    DEBUG_PRINT("Found BMP280 sensor! No Humidity available.");
    break;
  default:
    DEBUG_PRINT("Found UNKNOWN sensor! Error!");
  }
}

float getAltitude()
{
  return EnvironmentCalculations::Altitude(getTemperature(), EnvironmentCalculations::AltitudeUnit_Meters, SEALEVELPRESSURE_HPA, getTemperature(), EnvironmentCalculations::TempUnit_Celsius);
}

float getPressure()
{
  return (bme.pres() / 100.0F);
}

float getTemperature()
{
  return bme.temp();
}

float getHumidity()
{
  return bme.hum();
}

float getDewPoint()
{
  return EnvironmentCalculations::DewPoint(getTemperature(), getHumidity(), EnvironmentCalculations::TempUnit_Celsius);
}

float getHeadIndex()
{
  return EnvironmentCalculations::HeatIndex(getTemperature(), getHumidity(), EnvironmentCalculations::TempUnit_Celsius);
}



String getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}

String getBinName(String url)
{
  int index = 0;


  for (int i = 0; i < url.length(); i++)
  {
    if (url[i] == '/')
    {
      index = i;
    }
  }

  String binName = "";


  for (int i = index; i < url.length(); i++)
  {
    binName += url[i];
  }

  return binName;
}

String getHostName(String url)
{
  int index = 0;


  for (int i = 0; i < url.length(); i++)
  {
    if (url[i] == '/')
    {
      index = i;
    }
  }

  String hostName = "";


  for (int i = 0; i < index; i++)
  {
    hostName += url[i];
  }

  return hostName;
}

void update(char *payload)
{
  int contentLength = 0;
  bool isValidContentType = false;

  DynamicJsonDocument doc(200);
  deserializeJson(doc, payload);
  double FirmwareVer = doc["FirmwareVer"];
  String url = doc["url"];
  int port = 80;

  rainflowMQTT.publish("update", 2, true, "", 0, false, 0);

  if (FirmwareVer > FIRMWARE_VER)
  {
    String bin = getBinName(url);
    String host = getHostName(url);

    Serial.println("Connecting to: " + host);
    if (client.connect(host.c_str(), port))
    {


      Serial.println("Fetching Bin: " + bin);


      client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" +
                   "Cache-Control: no-cache\r\n" +
                   "Connection: close\r\n\r\n");

      unsigned long timeout = millis();

      while (client.available() == 0)
      {
        if (millis() - timeout > 5000)
        {
          Serial.println("Client Timeout !");
          client.stop();
          return;
        }
      }
      while (client.available())
      {

        String line = client.readStringUntil('\n');

        line.trim();






        if (!line.length())
        {

          break;
        }



        if (line.startsWith("HTTP/1.1"))
        {
          if (line.indexOf("200") < 0)
          {
            Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
            break;
          }
        }



        if (line.startsWith("Content-Length: "))
        {
          contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
          Serial.println("Got " + String(contentLength) + " bytes from server");
        }


        if (line.startsWith("Content-Type: "))
        {
          String contentType = getHeaderValue(line, "Content-Type: ");
          Serial.println("Got " + contentType + " payload.");
          if (contentType == "application/octet-stream")
          {
            isValidContentType = true;
          }
        }
      }
    }
    else
    {



      Serial.println("Connection to " + host + " failed. Please check your setup");

    }


    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));


    if (contentLength && isValidContentType)
    {

      bool canBegin = Update.begin(contentLength);
      if (canBegin)
      {
        Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
        size_t written = Update.writeStream(client);

        if (written == contentLength)
        {
          Serial.println("Written : " + String(written) + " successfully");
        }
        else
        {
          Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");

        }

        if (Update.end())
        {
          Serial.println("OTA done!");
          if (Update.isFinished())
          {
            Serial.println("Update successfully completed. Rebooting.");
            ESP.restart();
          }
          else
          {
            Serial.println("Update not finished? Something went wrong!");
          }
        }
        else
        {
          Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        }
      }
      else
      {



        Serial.println("Not enough space to begin OTA.");
        client.flush();
      }
    }
    else
    {
      Serial.println("There was no content in the response");
      client.flush();
    }
  }
}

void onMqttConnect(bool sessionPresent)
{
  DEBUG_PRINT("MQTT Connected. Session present:" + String(sessionPresent));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  DEBUG_PRINT("MQTT Disconnected.");

}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  DEBUG_PRINT("Subscribe acknowledged. PacketID:" + String(packetId));
}

void onMqttUnsubscribe(uint16_t packetId)
{
  DEBUG_PRINT("Unsubscribe acknowledged. PacketID:" + String(packetId));
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  DEBUG_PRINT("Message Received. ");
  DEBUG_PRINT("Topic: " + String(topic));
  DEBUG_PRINT("Payload: " + String(payload));

  if (String(topic) == "update")
  {
    update(payload);
  }
}

void onMqttPublish(uint16_t packetId)
{
  DEBUG_PRINT("Publish acknowledged. PacketID:" + String(packetId));
}


void modeCheck()
{
  DEBUG_PRINT("Checking Mode.");

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  float minFloodDepth = 10;
  float curFloodDepth = getDepth();
  unsigned long lastTipTime = 1.8e6;
  unsigned long currentTime = millis();
  float curBattLevel = getBatteryLevel();
  DEBUG_PRINT("Last Detected Millis: " + String(lastDetectedTipMillis) + " " + String(lastDetectedTipMillis2));
  DEBUG_PRINT("Current Time: " + String(currentTime));
  DEBUG_PRINT("Current Flood Depth: " + String(curFloodDepth));
  DEBUG_PRINT("Current Battery Level: " + String(curBattLevel));




  if (((curFloodDepth < minFloodDepth) && ((currentTime - lastDetectedTipMillis) >= lastTipTime) && ((currentTime - lastDetectedTipMillis2) >= lastTipTime)) || ((currentMode == 0) && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)))
  {
    mode_Standby();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime) || ((currentTime - lastDetectedTipMillis2) < lastTipTime)) && (curBattLevel > 20.00))
  {
    mode_ContinuousMonitoring();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime) || ((currentTime - lastDetectedTipMillis2) < lastTipTime)) && (curBattLevel <= 20.00))
  {
    mode_BatterySaver();
  }
}

void mode_Standby()
{
  DEBUG_PRINT("Standby Mode.");
  currentMode = 0;
  int minutesToSleep = 30;
  publishDataScheduler.disable();
  checkMode.disable();
  publishData();
  sleep(minutesToSleep * 60);
}

void mode_ContinuousMonitoring()
{
  DEBUG_PRINT("Mode: Continuous Monitoring");
  if (!publishDataScheduler.isEnabled() || currentMode != 1)
  {
    publishDataScheduler.disable();
    publishDataScheduler.setInterval(5 * 60e3);
    publishDataScheduler.enable();
  }
  if (!checkMode.isEnabled() || currentMode != 1)
  {
    checkMode.disable();
    checkMode.setInterval(30 * 60e3);
    checkMode.enable();
  }
  currentMode = 1;
}

void mode_BatterySaver()
{
  DEBUG_PRINT("Mode: Battery Saver");
  if (!publishDataScheduler.isEnabled() || currentMode != 2)
  {
    publishDataScheduler.disable();
    publishDataScheduler.setInterval(30 * 60e3);
    publishDataScheduler.enable();
  }
  if (!checkMode.isEnabled() || currentMode != 2)
  {
    checkMode.disable();
    checkMode.setInterval(15 * 60e3);
    checkMode.enable();
  }
  currentMode = 2;
}


void sleep(int time_to_sleep)
{

  esp_sleep_enable_ext1_wakeup(GPIO_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping for " + String(time_to_sleep) + " Seconds");
  rainflowMQTT.disconnect();
  DEBUG_PRINT("Disconnected from server.");
  wait(5000);
  Serial.flush();
  esp_deep_sleep_start();
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    DEBUG_PRINT("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    DEBUG_PRINT("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    DEBUG_PRINT("Wakeup caused by timer");
    break;
  default:
    DEBUG_PRINT("Wakeup was not caused by deep sleep:");
    DEBUG_PRINT(String(wakeup_reason));
    break;
  }
}

void wait(unsigned long interval)
{

  unsigned long time_now = millis();
  while (!(millis() - time_now >= interval))
  {
  }
}

void dataPublish()
{
  DynamicJsonDocument payloadData(1024);
  String payloadBuffer;
  String topic;
  int len;

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamIDData;

  DEBUG_PRINT("Retrieving data.");
  String unixTime = getUnixTime();
  DEBUG_PRINT("Current time: " + String(unixTime));
  JsonObject payload_Data = payloadData.createNestedObject("data");
  DEBUG_PRINT("Retrieving Latitude.");
  JsonObject objectLatitude = payload_Data.createNestedObject("LAT1");
  objectLatitude["time"] = unixTime;
  objectLatitude["value"] = gps.location.lat();

  DEBUG_PRINT("Retrieving Longitude.");
  JsonObject objectLongitude = payload_Data.createNestedObject("LNG1");
  objectLongitude["time"] = unixTime;
  objectLongitude["value"] = gps.location.lng();

  DEBUG_PRINT("Retrieving Altitude.");
  JsonObject objectAltitude = payload_Data.createNestedObject("ALT1");
  objectAltitude["time"] = unixTime;
  objectAltitude["value"] = getAltitude();

  DEBUG_PRINT("Retrieving Rain Rate.");
  JsonObject objectRainRate = payload_Data.createNestedObject("RR1");
  objectRainRate["time"] = unixTime;
  objectRainRate["value"] = rainfallRate();

  DEBUG_PRINT("Retrieving Rain Amount.");
  JsonObject objectRainAmount = payload_Data.createNestedObject("RA1");
  objectRainAmount["time"] = unixTime;
  objectRainAmount["value"] = rainfallAmount();
# 1082 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
  DEBUG_PRINT("Retrieving Temperature.");
  JsonObject objectTemp = payload_Data.createNestedObject("TMP1");
  objectTemp["time"] = unixTime;
  objectTemp["value"] = getTemperature();

  DEBUG_PRINT("Retrieving Pressure.");
  JsonObject objectPress = payload_Data.createNestedObject("PR1");
  objectPress["time"] = unixTime;
  objectPress["value"] = getPressure();

  DEBUG_PRINT("Retrieving Humidity.");
  JsonObject objectHumid = payload_Data.createNestedObject("HU1");
  objectHumid["time"] = unixTime;
  objectHumid["value"] = getHumidity();

  DEBUG_PRINT("Retrieving DewPoint.");
  JsonObject objectDew = payload_Data.createNestedObject("DP1");
  objectDew["time"] = unixTime;
  objectDew["value"] = getDewPoint();

  DEBUG_PRINT("Retrieving Heat Index.");
  JsonObject objectHeat = payload_Data.createNestedObject("HI1");
  objectHeat["time"] = unixTime;
  objectHeat["value"] = getHeadIndex();

  DEBUG_PRINT("Retrieving Battery Votlage.");
  JsonObject objectBatt = payload_Data.createNestedObject("BV1");
  objectBatt["time"] = unixTime;
  objectBatt["value"] = getBatteryVoltage();

  DEBUG_PRINT("Retrieving Flood Depth.");
  JsonObject objectFloodDepth = payload_Data.createNestedObject("FD1");
  objectFloodDepth["time"] = unixTime;
  objectFloodDepth["value"] = getDepth();

  serializeJson(payloadData, payloadBuffer);
  topic = "RAFT_Data";
  len = strlen(payloadBuffer.c_str());
  rainflowMQTT.publish(topic.c_str(), 2, false, payloadBuffer.c_str(), len, false, 0);
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  DEBUG_PRINT("payloadData Mem:" + String(payloadData.memoryUsage()));
  wait(2000);
}

void infoPublish()
{
  DynamicJsonDocument payloadData(1024);
  String payloadBuffer;
  String topic;
  int len;

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamIDInfo;

  DEBUG_PRINT("Retrieving info.");
  String unixTime = getUnixTime();
  DEBUG_PRINT("Current time: " + String(unixTime));
  JsonObject payload_Data = payloadData.createNestedObject("data");
  JsonObject objectFirmware = payload_Data.createNestedObject("FirmwareVer");
  objectFirmware["time"] = unixTime;
  objectFirmware["value"] = FIRMWARE_VER;

  JsonObject objectBoot = payload_Data.createNestedObject("bootCount");
  objectBoot["time"] = unixTime;
  objectBoot["value"] = bootCount;

  JsonObject objectDate = payload_Data.createNestedObject("RGDate");
  objectDate["time"] = unixTime;
  objectDate["value"] = rainGaugeDate;

  JsonObject objectTip = payload_Data.createNestedObject("tipCount");
  objectTip["time"] = unixTime;
  objectTip["value"] = tipCount;

  JsonObject objectHeight = payload_Data.createNestedObject("Height");
  objectHeight["time"] = unixTime;
  objectHeight["value"] = medianGetHeight;

  JsonObject objectMode = payload_Data.createNestedObject("Mode");
  objectMode["time"] = unixTime;
  objectMode["value"] = currentMode;

#ifdef MODEM_WIFI
  JsonObject objectWifi = payload_Data.createNestedObject("wifiRSSI");
  objectWifi["time"] = unixTime;
  objectWifi["value"] = getRSSI(ssid);
#endif

  serializeJson(payloadData, payloadBuffer);
  topic = "RAFT_Info";
  len = strlen(payloadBuffer.c_str());
  rainflowMQTT.publish(topic.c_str(), 2, false, payloadBuffer.c_str(), len, false, 0);
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  DEBUG_PRINT("payloadData Mem:" + String(payloadData.memoryUsage()));
  wait(2000);
}

void publishData()
{
  indicatorLED1(true);
  bool connected = false;

#ifdef MODEM_WIFI

  if (WiFi.status() != WL_CONNECTED)
  {
    connectWifi(ssid, wifi_pass);
    wait(5000);
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    connected = true;
  }
#endif
#ifdef MODEM_GSM
  connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
  gprsConnect();
#endif

  if (connected == true)
  {
    rainflowMQTT.connect();
    wait(5000);
    if (rainflowMQTT.connected())
    {
      dataPublish();
      infoPublish();
    }

    if (currentMode == 0)
    {
      DEBUG_PRINT("Subscribing");
      rainflowMQTT.subscribe("inbox", 2);
      rainflowMQTT.subscribe("update", 2);

      wait(15000);
    }

    rainflowMQTT.disconnect();
  }

#ifdef MODEM_WIFI
  disconnectWifi();
#endif
#ifdef MODEM_GSM
#endif

  indicatorLED1(false);
}

void indicatorLED(bool status)
{
  if (status == true)
  {
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, HIGH);
  }
  if (status == false)
  {
    digitalWrite(LEDPin, LOW);
    pinMode(LEDPin, INPUT);
  }
}

void indicatorLED1(bool status)
{
  if (status == true)
  {
    pinMode(LEDPin1, OUTPUT);
    digitalWrite(LEDPin1, HIGH);
  }
  if (status == false)
  {
    digitalWrite(LEDPin1, LOW);
    pinMode(LEDPin1, INPUT);
  }
}

void setup()
{
#ifdef DEBUG_MODE
  Serial.begin(115200);
  wait(1000);
#endif
  DEBUG_PRINT("System initialising");
  setCpuFrequencyMhz(80);
  bootCount++;
  print_wakeup_reason();
  esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause();
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
  {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();




    if (GPIO_reason != 0)
    {
      int pin = __builtin_ffsll(GPIO_reason) - 1;
      Serial.printf("Wake up from GPIO %d\n", pin);
    }
    else
    {
      Serial.printf("Wake up from GPIO\n");
    }
# 1301 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
    tipTime = 0;
  }

  if (!EEPROM.begin(64))
  {
    DEBUG_PRINT("Failed to initialise EEPROM");
    DEBUG_PRINT("Restarting...");
    wait(1000);
    ESP.restart();
  }

  pinMode(BATTERYPIN, INPUT);
  attachGPS();
  attachRainGauge();
  attachBarometer();
  attachUS();
  if (bootCount == 1)
  {
    wait(5000);
    getHeight();
    if (heightWrite == true)
      setHeight();
  }
  if ((wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) && (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER))
  {

    rainfallAmountReset();
    GPS_powerSaveMode();
  }
# 1341 "D:/Documents/PlatformIO/Projects/200629-111335-nodemcu-32s/src/RFLOW_Rain_Gauge_Benchmarking_API_Test.ino"
  rainflowMQTT.setClientId(clientID);
  rainflowMQTT.setCredentials(username, password);
  rainflowMQTT.setServer("rainflow.live", 1883);
  rainflowMQTT.onConnect(onMqttConnect);
  rainflowMQTT.onDisconnect(onMqttDisconnect);
  rainflowMQTT.onSubscribe(onMqttSubscribe);
  rainflowMQTT.onUnsubscribe(onMqttUnsubscribe);
  rainflowMQTT.onMessage(onMqttMessage);
  rainflowMQTT.onPublish(onMqttPublish);

  Runner.init();
  Runner.addTask(publishDataScheduler);
  Runner.addTask(checkMode);
  Runner.addTask(rainfallReset);
  Runner.addTask(setHeightTask);
  rainfallReset.enable();
  modeCheck();
}

void loop()
{
  Runner.execute();
}