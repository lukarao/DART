#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <Servo.h>

class WifiRcl {
  public:
		void setup();
    void update();
    void sendAlt(float alt);
    bool connected();

    float throttle = 0; //throttle stick value 0.0 (zero throttle/stick back) to 1.0 (full throttle/stick forward)
    float roll = 0; //roll stick value -1.0 (left) to 1.0 (right)
    float pitch = 0; //pitch stick value -1.0 (pitch up/stick back) to 1.0 (pitch down/stick forward)
    float yaw = 0; //yaw stick value -1.0 (left) to 1.0 (right)
    bool armed = false; //armed state (triggered by arm switch or stick commands)
  
  private:
    WiFiServer server = WiFiServer(2000);
    WiFiClient client;
    Servo armServo;
    unsigned long armActionStart = 0;
};
