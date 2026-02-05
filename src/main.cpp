#include <Arduino.h>
#include "function.h"
#include <Wire.h>
#include <MPU6050.h>


void setup() {
  initGyro();
}

void loop() {
  // se charger de la compensation température
  float angle = getAngle();
  Serial.printf("Angle: %.2f\n", angle);
  delay(5); // Rapidité de lecture
}