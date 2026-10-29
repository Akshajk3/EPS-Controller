#include <Arduino.h>
#include <TorqueSteer.h>

float Kp = 1.25f;
float Kd = 0.0f;
float Ki = 0.7f;

TorqueSteer tSteer(Kp, Kd, Ki);

void setup() {
  Serial.begin(9600);
  
  tSteer.begin();
  tSteer.setTargetAngle(45.0f);
}

void loop() {
  tSteer.update();
  delay(150);
}