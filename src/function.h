#ifndef FUNCTION_H
#define FUNCTION_H
#include "main.hpp"
#include <Arduino.h>

// Exemple de déclaration de fonction
void initGyro(void);
float getAngle(void);
void confGPIO(void);
void calibrageGyroscope(int number = 1000);


#endif