#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);  // Set Wi-Fi to station mode
  WiFi.begin();         // Start Wi-Fi (even without credentials)

  delay(100);  // Give it a moment to initialize

  String macAddress = WiFi.macAddress();
  Serial.println("ESP32 MAC Address: " + macAddress);
}

void loop() {
}
