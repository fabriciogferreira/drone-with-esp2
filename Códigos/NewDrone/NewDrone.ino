//------------------------------------|LIBRARIES|------------------------------------
#include <WiFi.h>

void StartStationMode() {
  WiFi.mode(WIFI_STA);
}

void setup() {
  StartStationMode();
}

void loop() {
  // put your main code here, to run repeatedly:

}
