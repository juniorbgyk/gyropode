#ifndef FUNCTION_H
#define FUNCTION_H
#include "main.hpp"
#include <Arduino.h>

// Exemple de déclaration de fonction
void initGyro(void);
float getAngle(void);
void confGPIO(void);
void calibrageGyroscope(int number = 1000);
void deplacement(bool mode, unsigned int vitesse1, unsigned int vitesse2 = 0);
float getVitesse(void);
void reception(char ch);
void serialEvent();

#endif