#include "wifi_rcl.h"

void WifiRcl::setup() {
  if (WiFi.softAP("drone-hotspot", "password")) {
    Serial.println("RCL: Hotspot initialized");
    this->server.begin();
    Serial.print("RCL: Server initialized at port 2000; local IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("RCL: Failed to initialize hotspot");
  }
}

void WifiRcl::update() {
  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.connected()) {
    if (client.available()) {
      Serial.print("RCL: Received byte: ");
      Serial.println(client.read());
    }
  }
}

bool WifiRcl::connected() {
  return client && client.connected();
} 
