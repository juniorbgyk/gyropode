#include "function.h"
#include <Arduino.h>  // utile si tu utilises digitalWrite, etc.
#include "main.hpp"
#include <Wire.h>
#include <MPU6050.h>

float angle = 0.0;
float confianceGyro = 0.98; // la alpha : rapidité détermination angle (gyro mesure vitesse rotation)
float confianceAcc = 0.02;  // correction pour avoir un angle plus fiable (accéléro mesure gravité pour savoir ou est le sol)
unsigned long lastTime;

// Paramètres : Tolérance de 1.5 à 2 degrés
const float tolerance = 0.5; 
float lastDisplayedAngle = 0.0;

MPU6050 mpu;

void initGyro() {
    //configuration nécessaire
    Serial.begin(115200);
    Wire.begin(); //initialisation du bus I2C


    // Configurer les broches en I/O
    pinMode(MOT1P, OUTPUT);
    pinMode(MOT1N, OUTPUT);
    pinMode(MOT2P, OUTPUT);
    pinMode(MOT2N, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(M1A, INPUT);
    pinMode(M1B, INPUT);
    pinMode(M2A, INPUT);
    pinMode(M2B, INPUT);


    //initialisation du MPU6050
    mpu.initialize(); 
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 non détecté !");
        while (1);
    }
    // --- Permet de partir de l'angle réel pour l'asservissement ---
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    angle = atan2(ay, az) * 180.0 / PI; //calcul de l'angle grace a la foce exercer sur l'axe Y et Z   
    lastTime = millis(); //récupère dernier valeur compter en ms (quand l'esp est alimenté il commence a compter le temps)
}

float getAngle() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Utilisation du Gyroscope Y
    float gyroY = gy / 131.0; // Conversion en degrés/s
    // Calcul de l'angle via l'Accéléromètre
    float angleAcc = atan2(ay, az) * 180.0 / PI;
    // Calcul du delta temps
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // temps écoulé depuis la dernière itération (en secondes)
    lastTime = now;
    // Filtre complémentaire
    angle = confianceGyro * (angle + gyroY * dt) + confianceAcc * angleAcc;
    // Configuration angle stable avec tolérance
    if (abs(angle - lastDisplayedAngle) >= tolerance) {
        lastDisplayedAngle = angle;
    }

    // Affichage
    // Serial.print("Angle Instantané: ");
    // Serial.print(angle);
    // Serial.print(" | Angle Stable: ");
    // Serial.println(lastDisplayedAngle);
    return lastDisplayedAngle;
}