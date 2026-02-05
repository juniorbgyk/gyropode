#include "function.h"
#include <Arduino.h>  // utile si tu utilises digitalWrite, etc.
#include "main.hpp"
#include <Wire.h>
#include <MPU6050.h>

float angle = 0.0;
int16_t ax, ay, az, gx, gy, gz;
long gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
//on est rapide mais imprécis
float confianceGyro = 0.02; // la alpha: rapidité détermination angle (gyro mesure vitesse rotation)
float confianceAcc = 0.98;  // correction pour avoir un angle plus fiable (accéléro mesure gravité pour savoir ou est le sol)
unsigned long lastTime;

// Paramètres : Tolérance de 1.5 à 2 degrés
const float tolerance = 0.5; // Tolérance en degrés pour considérer l'angle comme stable
float angletolerer = 0.0;

MPU6050 mpu;

void confGPIO(void){
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
}

void calibrageGyroscope(int number){
    // Calibrage du gyroscope
    for(int i = 0; i < number; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        gyroX_offset += gx;
        gyroY_offset += gy;
        gyroZ_offset += gz;
        delay(2);
    }
    gyroX_offset /= number;
    gyroY_offset /= number;
    gyroZ_offset /= number;
}

void initGyro() {
    confGPIO();
    //initialisation du MPU6050
    //CONFIGURATION MPU voir datasheet page 9 et 10
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ±500°/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // ±4g
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);             // Filtre BF 188Hz
    mpu.initialize(); 
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 non détecté !");
        while (1);
    }
    calibrageGyroscope(); //premet davoir une meilleur précision du gyroscope en compensant les biais de mesure
    // --- Permet de partir de l'angle réel pour l'asservissement ---
    mpu.getAcceleration(&ax, &ay, &az);
    angle = atan2(ay, ax) * 180.0 / PI; //calcul de l'angle grace a la foce exercer sur l'axe Y et Z   
    lastTime = millis(); //récupère dernier valeur compter en ms (quand l'esp est alimenté il commence a compter le temps)
}

float getAngle() {
    float gyroY;
    if (millis() >= 3000) // Si 3 seconde est écoulée début de tache
    {
        //on est précis mais plus lent
        confianceGyro = 0.98;
        confianceAcc = 0.02;
    }
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Utilisation du Gyroscope Y
    gyroY = gy - gyroY_offset; //mesure calibré
    gyroY = gyroY / 131.0; // Conversion en degrés/s
    // Calcul de l'angle via l'Accéléromètre
    float angleAcc = atan2(ay, ax) * 180.0 / PI;
    // Calcul du delta temps
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // temps écoulé depuis la dernière itération (en secondes)
    lastTime = now;
    // Filtre complémentaire
    angle = confianceGyro * (angle + gyroY * dt) + confianceAcc * angleAcc;
    // Configuration angle stable avec tolérance
    if (abs(angle - angletolerer) >= tolerance) {
        angletolerer = angle;
    }

    // Affichage
    // Serial.print("Angle Instantané: ");
    // Serial.print(angle);
    // Serial.print(" | Angle Stable: ");
    // Serial.println(angletolerer);
    return angletolerer;
}