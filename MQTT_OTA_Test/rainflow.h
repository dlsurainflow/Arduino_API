#ifndef rainflow_h
#define rainflow_h

#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include "PubSubClient.h"

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

#if defined (ESP8266) || defined (ESP32)
#include <functional>
#define MQTT_CALLBACK_SIGN std::function<void(String)> mqttCallback
#else
#define MQTT_CALLBACK_SIGN void (*mqttCallback)(String)
#endif


class RainFLOW {
  private:
    //Client* client;

    PubSubClient rainflowMQTT;    // Instance Creation of MQTT Client
    DynamicJsonDocument payloadData;
    MQTT_CALLBACK_SIGN;
    

  public:
    RainFLOW() : payloadData(1024) {}   //JSON Containing Array
    JsonObject payload_Data = payloadData.createNestedObject("data");
    void connectServer(const char* clientID, const char* username, const char* password);
    bool connectMqtt(const char* clientID, const char* username, const char* password);
    void callback(char* topic, byte * payload, unsigned int len);
    RainFLOW& setCallback(MQTT_CALLBACK_SIGN);
    void addData(const char* dataName, String dataPayload, String dataTime);
    void publishData(const char* clientID, const char* username, const char* password, const char* streamID);
    void rainflowCallback(char* topic, byte* payload, unsigned int len);
    void rainflow(Client& client);
    void keepAlive();
    void disconnect();


};


#endif
