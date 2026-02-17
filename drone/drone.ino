#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  if (WiFi.softAP("drone-hotspot", "password")) {
    Serial.println("Hotspot initialized");
  } else {
    Serial.println("Failed to initialize hotspot");
  }
}

void loop() {}
