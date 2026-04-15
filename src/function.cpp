#include "function.h"
#include <Arduino.h>  // utile si tu utilises digitalWrite, etc.
#include "main.hpp"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

// Déclarations des encodeurs de roue
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// Variables globales de contrôle et de capteur
float kp = 71.7;           // coefficient P du régulateur d'équilibre
float kd = 2.025;          // coefficient D du régulateur d'équilibre
float v_kd = 0;            // coefficient D pour la vitesse (si utilisé)
float v_kp = 44.9;         // coefficient P pour la vitesse (si utilisé)
float offsetDZ1 = 585.5;   // offset PWM pour moteur 1, pour compensation frottement secs
float offsetDZ2 = 601.0;   // offset PWM pour moteur 2, pour compensation frottement secs
float gyroZ;               // angle calculé à partir du gyroscope
float angleAcc;            // angle calculé à partir de l'accéléromètre
float offsetAngle = -4.52; // compensation d'offset de l'angle

// Variables de vitesse
float vitesse1;
float vitesse2;
float vitesse;
float last_vitesse1;
float last_vitesse2;
int32_t lastIncrement1 = 0; // compteur précédent encodeur 1
int32_t lastIncrement2 = 0; // compteur précédent encodeur 2

// Variables du MPU6050
static float angle = 0.0; // angle initial mesuré à l'allumage
float ax, ay, gz;         // valeurs d'accélération et de gyroscope
char FlagCalcul = 0;      // drapeau non utilisé ici, possiblement réservé pour une extension
float entrerFiltre;
float anglefiltrer = 0;

// constantes de filtrage
float Te = 5;    // période d'échantillonnage en ms
float Tau = 200; // constante de temps du filtre en ms
float Tauvitesse = 88; // constante de temps du filtre de vitesse
float A, B, C, D;     // coefficients du filtre numérique


Adafruit_MPU6050 mpu; // instance du capteur MPU6050

void confGPIO(void) {
    // Initialisation de la communication série
    Serial.begin(115200);

    // Initialisation du bus I2C pour le MPU6050
    Wire.begin(21, 22);

    // Configuration des broches en entrée / sortie
    pinMode(LED, OUTPUT);
    pinMode(VBAT_pin, INPUT);
    pinMode(M1A, INPUT);
    pinMode(M1B, INPUT);
    pinMode(M2A, INPUT);
    pinMode(M2B, INPUT);

    // Initialisation des canaux PWM pour les moteurs
    ledcSetup(CanalM1P, 20000, 10);
    ledcSetup(CanalM1N, 20000, 10);
    ledcSetup(CanalM2P, 20000, 10);
    ledcSetup(CanalM2N, 20000, 10);
    ledcAttachPin(MOT1P, CanalM1P);
    ledcAttachPin(MOT1N, CanalM1N);
    ledcAttachPin(MOT2P, CanalM2P);
    ledcAttachPin(MOT2N, CanalM2N);
}

void calculCoeffFiltre(void) {
    // Calcul des coefficients pour les filtres passe-bas
    A = 1 / (1 + Tau / Te);
    B = Tau / Te * A;
    C = 1 / (1 + (Tauvitesse) / Te);
    D = (Tauvitesse) / Te * C;
}

void initGyro() {
    // Initialisation matérielle commune
    confGPIO();

    // Attente de l'ouverture du port série
    while (!Serial) delay(10);

    // Initialisation du capteur MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    // Réglage de la gamme de l'accéléromètre
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

    // Lecture initiale de l'accéléromètre pour définir un angle de référence
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ay = a.acceleration.y;
    ax = a.acceleration.x;
    angle = atan2(ay, ax) * 180.0 / PI; // angle initial en degrés

    // Calcul des coefficients du filtre
    calculCoeffFiltre();

    // Initialisation des encodeurs et remise à zéro des compteurs
    encoder1.attachFullQuad(M1A, M1B);
    encoder2.attachFullQuad(M2A, M2B);
    encoder1.setCount(0);
    encoder2.setCount(0);
}

float getAngle() {
    // Lecture des mesures du MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ay = a.acceleration.y;
    ax = a.acceleration.x;
    gz = g.gyro.z;

    // Calcul de la composante gyroscopique en degrés
    gyroZ = -(gz * Tau / 1000.0) * 180.0 / PI;

    // Calcul de l'angle à partir de l'accéléromètre
    angleAcc = atan2(ay, ax) * 180.0 / PI;

    // Fusion des deux estimations avec un filtre simple
    entrerFiltre = gyroZ + angleAcc;
    anglefiltrer = A * entrerFiltre + B * anglefiltrer;

    return anglefiltrer;
}

void deplacement(bool mode, signed int vitesse1, signed int vitesse2) {
    // Limitation des consignes de vitesse à [-400, 400]
    if (vitesse1 > 400) {
        vitesse1 = 400;
    }
    if (vitesse1 < -400) {
        vitesse1 = -400;
    }
    if (vitesse2 > 400) {
        vitesse2 = 400;
    }
    if (vitesse2 < -400) {
        vitesse2 = -400;
    }

    if (!mode) {
        // Mode de contrôle commun des deux roues (même vitesse appliquée aux deux moteurs)
        if (vitesse1 >= 0) {
            ledcWrite(CanalM1P, offsetDZ1 + vitesse1);
            ledcWrite(CanalM1N, 0);
            ledcWrite(CanalM2N, offsetDZ2 + vitesse1);
            ledcWrite(CanalM2P, 0);
        } else {
            ledcWrite(CanalM1N, offsetDZ1 + abs(vitesse1));
            ledcWrite(CanalM1P, 0);
            ledcWrite(CanalM2P, offsetDZ2 + abs(vitesse1));
            ledcWrite(CanalM2N, 0);
        }
    } else {
        // Mode de contrôle distinct des deux roues
        if (vitesse1 >= 0) {
            ledcWrite(CanalM1P, offsetDZ1 + vitesse1);
            ledcWrite(CanalM1N, 0);
        } else {
            ledcWrite(CanalM1N, offsetDZ1 + abs(vitesse1));
            ledcWrite(CanalM1P, 0);
        }
        if (vitesse2 >= 0) {
            ledcWrite(CanalM2N, offsetDZ2 + vitesse2);
            ledcWrite(CanalM2P, 0);
        } else {
            ledcWrite(CanalM2P, offsetDZ2 + abs(vitesse2));
            ledcWrite(CanalM2N, 0);
        }
    }
}

float getVitesse(void) {
    static float vitesseFiltrer;
    int32_t increment1, increment2;

    // Lecture des compteurs d'encodeur
    increment1 = encoder1.getCount() / 2;
    increment2 = encoder2.getCount() / 2;

    // Conversion des incréments en vitesse linéaire (m/s)
    vitesse1 = (((float)(increment1 - lastIncrement1) / PPR) * (rayonRoue * 2 * PI)) / 0.005;
    vitesse2 = (((float)(increment2 - lastIncrement2) / PPR) * (rayonRoue * 2 * PI)) / 0.005;

    // Mémorisation des compteurs pour la prochaine lecture
    lastIncrement1 = increment1;
    lastIncrement2 = increment2;

    // Calcul de la vitesse du gyropode à partir des deux roues
    vitesse = (-(vitesse1 - vitesse2)) / 2.0;

    // Filtrage de la vitesse
    vitesseFiltrer = C * vitesse + D * vitesseFiltrer;

    return vitesseFiltrer;
}

void reception(char ch) {
    // Traitement du flux série caractère par caractère
    static String chaine = "";
    String commande;
    String valeur;
    float param;
    int index, length;

    if ((ch == 13) or (ch == 10)) {
        // Fin de ligne détectée : traiter la commande complète
        index = chaine.indexOf(' ');
        length = chaine.length();
        if (index == -1) {
            commande = chaine;
            valeur = "";
        } else {
            commande = chaine.substring(0, index);
            valeur = chaine.substring(index + 1, length);
        }

        // Mise à jour des paramètres selon la commande reçue
        if (commande == "Kp") {
            kp = valeur.toFloat();
        }
        if (commande == "Kd") {
            kd = valeur.toFloat();
        }
        if (commande == "vKp") {
            v_kp = valeur.toFloat();
        }
        if (commande == "vKd") {
            v_kd = valeur.toFloat();
        }
        if (commande == "un") {
            offsetDZ1 = valeur.toFloat();
        }
        if (commande == "dx") {
            offsetDZ2 = valeur.toFloat();
        }
        if (commande == "Tau") {
            Tau = valeur.toFloat();
            calculCoeffFiltre();
        }
        if (commande == "Te") {
            Te = valeur.toInt();
            calculCoeffFiltre();
        }
        if (commande == "Off") {
            offsetAngle = valeur.toFloat();
        }

        // Réinitialiser la chaîne pour la prochaine commande
        chaine = "";
    } else {
        // Ajouter le caractère à la commande en cours
        chaine = chaine + ch;
    }
}

void serialEvent() {
    // Lecture de l'entrée série et envoi de chaque octet vers le parser
    while (Serial.available() > 0) {
        reception(Serial.read());
    }
}
