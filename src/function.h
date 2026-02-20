#ifndef FUNCTION_H
#define FUNCTION_H
#include "main.hpp"
#include <Arduino.h>

// Exemple de déclaration de fonction
void initGyro(void);
float getAngle(void);
void confGPIO(void);
void deplacement(bool mode, signed int vitesse1, signed int vitesse2 = 0);
float getVitesse(void);
void reception(char ch);
void serialEvent();
void calculCoeffFiltre(void);

#endif