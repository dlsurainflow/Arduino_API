#include "rainflow.h"
//#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>


#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) Serial.println(x)
#endif

//AsyncMqttClient rainflowMQTT;


void RainFLOW::connectServer(const char* clientID, const char* username, const char* password) {
  rainflowMQTT.connect();
  wait(1000);
  //rainflowMQTT.setCallback(rainflowCallback);
  //rainflowMQTT.setCallback([this] (char* topic, byte * payload, unsigned int length) {
  //this->callback(topic, payload, length);
  //});
  //DEBUG_PRINT("Connecting to RainFLOW Server.");
  //while (connectMqtt(clientID, username, password) == false) continue;
}

// bool RainFLOW::connectMqtt(const char* clientID, const char* username, const char* password) {
//   //  int i = 0;
//   //  while (!rainflowMQTT.connect(clientID, username, password)) {
//   //    DEBUG_PRINT(".");
//   //    wait(50);
//   //    if (!(i%25))  return false;
//   //    i++;
//   //  }
//   //  DEBUG_PRINT("Connected to server!");
//   //  rainflowMQTT.subscribe("Device");                  // Subscribe to device management channel
//   //  DEBUG_PRINT("Subscribed to: Device");
//   //  return rainflowMQTT.connected();
//   //  wait(50);
// }

// void rainflowCallback(char* topic, byte* payload, unsigned int len) {
//   DEBUG_PRINT("Received message: ");
//   Serial.write(payload, len);
// }

void RainFLOW::addData(const char* dataName, String dataPayload, String dataTime) {
  JsonObject nestedObject = payload_Data.createNestedObject(dataName);
  nestedObject["time"] = dataTime;
  nestedObject["value"] = dataPayload;
}


void RainFLOW::publishData(const char* clientID, const char* username, const char* password, const char* streamID) {
  //  while (connectMqtt(clientID, username, password) == false) continue;
  // char topicBuffer[1024];
  String payloadBuffer;
  String topic;
  int len;

  // const char* topicData = "data";

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamID;


  serializeJson(payloadData, payloadBuffer);
  topic =   "/RAFT_Data";
  len       = strlen(payloadBuffer.c_str());                                  // Calculates Payload Size
  rainflowMQTT.publish(topic.c_str(), 1, false, payloadBuffer.c_str(), len, false, 0);            // Publishes payload to server
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  payloadData.clear();                                                        // Clear JSON Documents
  payloadData.clear();
  wait(5000);
}

void RainFLOW::disconnect() {
  rainflowMQTT.disconnect();
}

void RainFLOW::rainflow(const char* clientID, const char* username, const char* password) {
  //  rainflowMQTT.setClient(client);
  rainflowMQTT.setClientId(clientID);
  rainflowMQTT.setCredentials(username, password);
  rainflowMQTT.setServer("rainflow.live", 1883);
  rainflowMQTT.onConnect(onMqttConnect);
  rainflowMQTT.onDisconnect(onMqttDisconnect);
  rainflowMQTT.onSubscribe(onMqttSubscribe);
  rainflowMQTT.onUnsubscribe(onMqttUnsubscribe);
  rainflowMQTT.onMessage(onMqttMessage);
  rainflowMQTT.onPublish(onMqttPublish);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  //  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  //  Serial.print("Subscribing at QoS 2, packetId: ");
  //  Serial.println(packetIdSub);
  //  mqttClient.publish("test/lol", 0, true, "test 1");
  //  Serial.println("Publishing at QoS 0");
  //  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  //  Serial.print("Publishing at QoS 1, packetId: ");
  //  Serial.println(packetIdPub1);
  //  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  //  Serial.print("Publishing at QoS 2, packetId: ");
  //  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// RainFLOW& RainFLOW::setCallback(MQTT_CALLBACK_SIGN) {
//   this->mqttCallback = mqttCallback;
//   return *this;
// }

// // void RainFLOW::callback(char* topic, byte* payload, unsigned int length) {
//   //
//   //  //  Generating String from received payload
//   //  String message = String();
//   //  for (int i = 0; i < length; i++) {
//   //    char input_char = (char)payload[i];
//   //    message += input_char;
//   //  }
//   //  //  Converting String to JSON Array
//   //  char jsonChar[100];
//   //  message.toCharArray(jsonChar, message.length() + 1);
//   //  StaticJsonBuffer<500> jsonBuffer;
//   //  JsonObject& root = jsonBuffer.parseObject(jsonChar);
//   //  String Topic = root["TOPIC"];
//   //
//   //  //  Reactions on received message
//   //
//   //  if (Topic == "SYSTEM")
//   //  {
//   //    String Payload = root["PAYLOAD"];
//   //
//   //    if (Payload == "INFO")
//   //    {
//   //      systemInfo();
//   //    }
//   //    else if (Payload == "NTP")
//   //    {
//   //      unsigned long _NTP = root["NTP"];
//   //      NTP = ((_NTP + 1000) - millis());
//   //      _Sunrise = root["SUNRISE"];
//   //      _Sunset = root["SUNSET"];
//   //      ReturnACK(String(CLIENT_ID) + " [" + Payload + " OK]");
//   //      WriteSPIFFS("Time", _Time);
//   //    }
//   //    else if (Payload == "OTA")
//   //    {
//   //      Publish("debug", String(CLIENT_ID) + " [" + Payload + " OK]");
//   //      String Link = "/" + String(CLIENT_ID) + ".bin";
//   //      ESPhttpUpdate.update(MQTT_SERVER, OTA_PORT, Link);
//   //    }
//   //    else if (Payload == "REBOOT")
//   //    {
//   //      Publish("debug", String(CLIENT_ID) + " [" + Payload + " OK]");
//   //      ESP.restart();
//   //    }
//   //  }
//   //
//   //  else if (Topic == "GPIO")
//   //  {
//   //    int Pin = root["PIN"];
//   //    int Value = root["VALUE"];
//   //    setOverride(Pin, Value);
//   //    WriteGPIO(Pin, Value);
//   //  }
//   //
//   //  else if (Topic == "OVERRIDE?")
//   //  {
//   //    int Pin = root["PIN"];
//   //    Publish("debug", "OVERRIDE[" + String(gpio_override[Pin]) + "]");
//   //  }
//   //  else if (Topic == "GPIO?")
//   //  {
//   //    int Pin = root["PIN"];
//   //    Publish("debug", "Pin[" + String(Pin) + " " + String(digitalRead(Pin)) + "]");
//   //  }
//   //  else if (Topic == "SPIFF")
//   //  {
//   //    String Object = root["OBJECT"];
//   //    int Value = root["VALUE"];
//   //    WriteSPIFFS(Object, Value);
//   //    Publish("debug", "SPIFF[" + Object + " " + String(Value) + "]");
//   //  }
//   //  else if (Topic == "SPIFF?")
//   //  {
//   //    String Object = root["OBJECT"];
//   //    int Value = ReadSPIFFS(Object);
//   //    Publish("debug", "SPIFF?[" + Object + " " + String(Value) + "]");
//   //  }
//   //  else
//   //  {
//   //    if (mqttCallback)
//   //    {
//   //      mqttCallback(message);
//   //    } else {
//   //      String Payload;
//   //      root.printTo(Payload);
//   //      Publish("debug", String(CLIENT_ID) + " [COMMAND INVALID]");
//   //    }
//   //  }
// }

// void RainFLOW::keepAlive() {
//   //  rainflowMQTT.loop();
// }

void RainFLOW::wait(unsigned long interval) {
  // In milliseconds (1s == 1000 ms)
  unsigned long time_now = millis();
  while (!(millis() - time_now >= interval)) {
  }
}
