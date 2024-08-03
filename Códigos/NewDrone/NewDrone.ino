//------------------------------------|LIBRARIES|------------------------------------
#include <WiFi.h>
#include <esp_now.h>

void StartStationMode() {
  WiFi.mode(WIFI_STA);
}

void StartEspNow() {
  if (!(esp_now_init() == ESP_OK)) {
    ESP.restart();
  }
}

void setup() {
  StartStationMode();
  StartEspNow();
}

void loop() {
  // put your main code here, to run repeatedly:

}
