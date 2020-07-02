#ifndef rainflow_h
#define rainflow_h

#include <HardwareSerial.h>
#include <ArduinoJson.h>
//#include "PubSubClient.h
#include "AsyncMqttClient.h"
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

// #if defined (ESP8266) || defined (ESP32)
// #include <functional>
// #define MQTT_CALLBACK_SIGN std::function<void(String)> mqttCallback
// #else
// #define MQTT_CALLBACK_SIGN void (*mqttCallback)(String)
// #endif


class RainFLOW {
  private:
    //Client* client;

    AsyncMqttClient rainflowMQTT;    // Instance Creation of MQTT Client
    DynamicJsonDocument payloadData;
    // MQTT_CALLBACK_SIGN;
    

  public:
    RainFLOW() : payloadData(2048) {}   //JSON Containing Array
    JsonObject payload_Data = payloadData.createNestedObject("data");
    void connectServer(const char* clientID, const char* username, const char* password);
    // bool connectMqtt(const char* clientID, const char* username, const char* password);
    // void callback(char* topic, byte * payload, unsigned int len);
    // RainFLOW& setCallback(MQTT_CALLBACK_SIGN);
    void addData(const char* dataName, String dataPayload, String dataTime);
    void publishData(const char* clientID, const char* username, const char* password, const char* streamID);
    // void rainflowCallback(char* topic, byte* payload, unsigned int len);
    void rainflow(const char* clientID, const char* username, const char* password);
    void disconnect();
    // void keepAlive();
    void wait(unsigned long interval);

};
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

#endif
