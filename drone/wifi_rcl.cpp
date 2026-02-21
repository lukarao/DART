#include "wifi_rcl.h"

const int ARM_PIN = 16; // GPIO27
const unsigned long ARM_ACTION_DURATION = 1500; // how long it takes arm to open/close in milliseconds

void WifiRcl::setup() {
  if (WiFi.softAP("drone-hotspot", "password")) {
    Serial.println("RCL: Hotspot initialized");
    this->server.begin();
    Serial.print("RCL: Server initialized at port 2000; local IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("RCL: Failed to initialize hotspot");
  }

  this->armServo.attach(ARM_PIN);
}

void WifiRcl::update() {
    this->throttle = 0.5;
    this->roll = 0;
    this->pitch = 0;
    this->yaw = 0;

  WiFiClient newClient = server.available();
  if (newClient) {
    Serial.println("RCL: Client connected");
    client = newClient;
  }

  if (client && !client.connected()) {
    Serial.println("RCL: Client disconnected");
    client = nullptr;
  }

  if (client && client.connected()) {
    if (client.available() >= 5) {
      uint8_t data[5];
      client.read(data, 5);

      this->armed = data[0] > 128;
      if (this->armState != data[1]) {
        this->armState = data[1];
        armActionStart = millis();
        this->armServo.write(this->armState > 128 ? 180 : 0);
      }
      this->throttle = data[2] / 255.0f;
      this->roll = (data[3] / 255.0f) * 2.0f - 1.0f;
      this->pitch = (data[4] / 255.0f) * 2.0f - 1.0f;
    }
  }

  if (millis() - armActionStart >= ARM_ACTION_DURATION) {
    this->armServo.write(90);
  }
}

void WifiRcl::sendAlt(float alt) {
  if (client && client.connected()) {
    uint8_t altData[4];
    memcpy(altData, &alt, sizeof(float));
    client.write(altData, 4);
  }
}

bool WifiRcl::connected() {
  return client && client.connected();
} 
