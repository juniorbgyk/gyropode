#include <Arduino.h>
#include "function.h"
#include <Wire.h>
#include <MPU6050.h>


void setup() {
  initGyro();
}

void loop() {
  float angle = getAngle();
  Serial.printf("Angle: %f\n", angle);
  delay(5); // Rapidité de lecture
}