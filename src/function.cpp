#include "function.h"
#include <Arduino.h>  // utile si tu utilises digitalWrite, etc.
#include "main.hpp"

void InitGPIO() {
    // Configurer les broches de sortie
    pinMode(MOT1P, OUTPUT);
    pinMode(MOT1N, OUTPUT);
    pinMode(MOT2P, OUTPUT);
    pinMode(MOT2N, OUTPUT);
    pinMode(LED, OUTPUT);

    // Configurer les broches d'entrée
    pinMode(M1A, INPUT);
    pinMode(M1B, INPUT);
    pinMode(M2A, INPUT);
    pinMode(M2B, INPUT);
}