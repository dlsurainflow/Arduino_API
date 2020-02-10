#include "rainflow.h"
#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>

PubSubClient rainflowMQTT;



void RainFLOW::connectServer(const char* APIKey) {
  rainflowMQTT.setServer("test.mosquitto.org", 1883);
  rainflowMQTT.setCallback(rainflowCallback);
  //  Serial.println("Connecting test.");
  DEBUG_PRINT("Connecting to RainFLOW Server at ");
  while (connectMqtt(APIKey) == false) continue;
}

bool RainFLOW::connectMqtt(const char* APIKey) {
  if (!rainflowMQTT.connect(APIKey)) {
    DEBUG_PRINT(".");
    return false;
  }
  DEBUG_PRINT("Connected to server!");
  char topic[45];
  strcpy(topic, "data/");
  strcat(topic, APIKey);
  rainflowMQTT.subscribe(topic);
  return rainflowMQTT.connected();
  DEBUG_PRINT("Subscribed to: " + String(topic));


}


void rainflowCallback(char* topic, byte* payload, unsigned int len) {
  DEBUG_PRINT("Received message: ");
  Serial.write(payload, len);
}

void RainFLOW::addData(String topic, String payload) {
  payloadData[topic] = payload;
}


void RainFLOW::publishData(const char* APIKey) {
  char topicBuffer[1024];
  String payloadBuffer;
  String topic;
  int len;
  //
  //  payloadData["API_Key"]            = APIKey;
  //  payloadData["Date"]               = "";
  //  payloadData["Time"]               = "";
  //  payloadData["Latitude"]           = "14.5647";
  //  payloadData["Longtitude"]         = "120.9932";
  //  payloadData["Altitude"]           = "64";
  //  payloadData["floodDepth"]         = "0";
  //  payloadData["rainfallAmount"]     = rainfallAmount;
  //  payloadData["batteryPercentage"]  = getBatteryPercent();
  //  payloadData["batteryVoltage"]     = getBatteryVoltage();
  //  serializeJson(payloadData, payloadBuffer);

  serializeJson(payloadData, payloadBuffer);
  topic =   "data/";
  topic  +=  APIKey;
  len       = strlen(payloadBuffer.c_str());

  rainflowMQTT.publish(topic.c_str(), payloadBuffer.c_str(), len);
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payload));

}


void RainFLOW::rainflow(Client& client) {
  rainflowMQTT.setClient(client);
}
