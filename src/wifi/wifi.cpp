#include <Arduino.h>
#include <WiFi.h>
#include <ESP.h>
#include <ESPmDNS.h>

void wifi_connect(const char* ssid, const char* password, const char* host)
{
     // Connect to WiFi network
    WiFi.begin(ssid, password);
    Serial.println("ESP32 Web OTA Update v1.0b");
    Serial.printf("Chip Rev: %d\n\r",ESP.getChipRevision());
    Serial.printf("SDK: %s\n\r",ESP.getSdkVersion());
    Serial.printf("CPU: %d MHz\n\r",ESP.getCpuFreqMHz());
    Serial.printf("Heap: %d\n\r",ESP.getFreeHeap());
    Serial.print("Booting");
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) {
        Serial.println("Error setting up MDNS responder!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");
}