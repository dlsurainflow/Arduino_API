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


class RainFLOW {
  private:
    //Client* client;

    PubSubClient rainflowMQTT;    // Instance Creation of MQTT Client
    DynamicJsonDocument payloadData;
    

  public:
    RainFLOW() : payloadData(1024) {}   //JSON Containing Array
    JsonObject payload_Data = payloadData.createNestedObject("data");
    void connectServer(const char* clientID, const char* username, const char* password);
    bool connectMqtt(const char* clientID, const char* username, const char* password);
    void addData(const char* dataName, String dataPayload, String dataTime);
    void publishData(const char* clientID, const char* username, const char* password, const char* streamID);
    void rainflowCallback(char* topic, byte* payload, unsigned int len);
    void rainflow(Client& client);
    void disconnect();


};
void rainflowCallback(char* topic, byte * payload, unsigned int len);

#endif
