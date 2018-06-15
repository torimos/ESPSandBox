#include "httpota.h"
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <Update.h>
#include <esp_wps.h>
#include <esp_image_format.h>

int updateState = 0;
void httpota_init(AsyncWebServer *server)
{
    SPIFFS.begin(true);  
    server->addHandler(new SPIFFSEditor(SPIFFS, "admin","admin"));
    server->serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server->on("/update", HTTP_POST, 
      [](AsyncWebServerRequest *request){
        bool fail = (Update.hasError() || updateState < 0); 
        AsyncWebServerResponse *response = request->beginResponse(fail?500:200, "text/plain", fail?"FAIL":"OK");
        response->addHeader("Connection","close");
        request->send(response);
        auto timeout = millis() + 500;
        while(millis()<=timeout)
        {
          delay(1);
        }
        esp_wifi_wps_disable();
        ESP.restart();
      }, 
      [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
        if (len == 0)
          updateState = -1;

        if(!index) {
          updateState = 0;
          Serial.printf("Starting firmware update: %s\n\r", filename.c_str());
          if (data[0] == ESP_IMAGE_HEADER_MAGIC) {
            //start with max available size
            if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
              Serial.println("Firmware update error");
              Update.printError(Serial);
              updateState = -1;
            }
          }
          else {
            // AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "FAIL");
            // response->addHeader("Connection","close");
            // request->send(response);
            updateState = -1;
          }
        }
        if (updateState<0)
          return;
         /* flashing firmware to ESP*/
        if(Update.write(data, len) != len){
          Serial.println("Firmware update error");
          Update.printError(Serial);
          updateState = -1;
        }
        else {
          Serial.printf("Progress: %u\r", index+len);
        }
        if (final) {
          Serial.println();
          if(Update.end(true)){ //true to set the size to the current progress
            Serial.printf("Update Success: %u\n\rRebooting...\n\r", (index+len));
          } else {
            Serial.println("Firmware update error");
            Update.printError(Serial);
            updateState = -1;
          }
        }
      });
    server->begin();
}