

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "wifi/wifi.h"
#include "ota/httpota.h"
#include "led/led.h"
#include <typeinfo>  //for 'typeid' to work  
#include <string.h>  //for 'typeid' to work  

const char* host = "esp32webupdate";
const char* ssid = "zzi-net";
const char* password = "smarthomeiot2018";
const char * mqttHost = "192.168.0.199";

uint mqttPort = 1883;

AsyncWebServer server(80);
AsyncMqttClient mqttClient;

long timeout = 0;
bool power = false;
uint prev_brightness = 1, brightness = 0;
char dsn[32] = {};

void onMqttConnect(bool sessionPresent) {
    Serial.println("Connected to MQTT.");
    mqttClient.subscribe("esp32/command", 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    auto v = String(topic);
    if (v == "esp32/command")
    {
        Serial.println(payload);
        StaticJsonBuffer<256> jsonBuffer;
        JsonObject &root = jsonBuffer.parseObject(payload);
        if (root.success()) {
            brightness = root["led"];
            power = root["power"];
            prev_brightness = 0;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    led_init();
    wifi_connect(ssid, password, host);
    httpota_init(&server);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(mqttHost, mqttPort);
    mqttClient.connect();

    uint64_t mac = ESP.getEfuseMac();
    sprintf(dsn, "%04X%08X", (uint16_t)(mac>>32), (uint32_t)mac);
}

void loop()
{
    if (!power)
    {
        led_write(0);
    }
    else if (prev_brightness != brightness)
    {
        led_write(brightness);
        prev_brightness = brightness;
    }

    if (millis() > timeout)
    {
        if (mqttClient.connected())
        { 
            String payload;
            StaticJsonBuffer<256> jsonBuffer;
            JsonObject &root = jsonBuffer.createObject();
            JsonObject &header = jsonBuffer.createObject();
            JsonObject &body =  jsonBuffer.createObject();
            root["header"] = header;
            root["body"] = body;
            header["dsn"] = dsn;
            body["hall"] = hallRead();
    
            root.prettyPrintTo(payload);
            mqttClient.publish("esp32/events", 0, true, payload.c_str());
        }
        timeout = millis() + 1000;
    }
}