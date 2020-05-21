#include "rainflow.h"
#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>


#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) Serial.println(x)
#endif

PubSubClient rainflowMQTT;


void RainFLOW::connectServer(const char* clientID, const char* username, const char* password) {
  rainflowMQTT.setServer("rainflow.live", 1883);
  //  rainflowMQTT.setCallback(rainflowCallback);
  DEBUG_PRINT("Connecting to RainFLOW Server.");
  while (connectMqtt(clientID, username, password) == false) continue;
}

bool RainFLOW::connectMqtt(const char* clientID, const char* username, const char* password) {
  if (!rainflowMQTT.connect(clientID, username, password)) {
    Serial.println(!rainflowMQTT.connect(clientID, username, password));
    DEBUG_PRINT(".");
    delay(50);
    return false;
  }
  DEBUG_PRINT("Connected to server!");
  rainflowMQTT.subscribe("/RAFT");                  // Subscribe to device management channel
  DEBUG_PRINT("Subscribed to: /RAFT_Data");
  return rainflowMQTT.connected();
  delay(50);
}


void rainflowCallback(char* topic, byte* payload, unsigned int len) {
  DEBUG_PRINT("Received message: ");
  Serial.write(payload, len);
}

void RainFLOW::addData(const char* dataName, String dataPayload, String dataTime) {
  JsonObject nestedObject = payload_Data.createNestedObject(dataName);
  nestedObject["time"] = dataTime;
  nestedObject["value"] = dataPayload;
  //  merge(nestedObject, payloadWrap);
  //  merge(nestedObject, payloadData);
  //  payloadData[dataName] = payload;                                             // Add payload to JSON variable
}


void RainFLOW::publishData(const char* clientID, const char* username, const char* password, const char* streamID) {
  while (connectMqtt(clientID, username, password) == false) continue;
  char topicBuffer[1024];
  String payloadBuffer;
  String topic;
  int len;

  const char* topicData = "data";

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamID;


  serializeJson(payloadData, payloadBuffer);
  topic =   "/RAFT_Data";
  len       = strlen(payloadBuffer.c_str());                                  // Calculates Payload Size
  rainflowMQTT.publish(topic.c_str(), payloadBuffer.c_str(), len);            // Publishes payload to server
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  payloadData.clear();                                                        // Clear JSON Documents
  delay(5000);
}


void RainFLOW::rainflow(Client& client) {
  rainflowMQTT.setClient(client);
}
