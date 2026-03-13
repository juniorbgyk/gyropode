#include "function.h"
#include <Arduino.h>  // utile si tu utilises digitalWrite, etc.
#include "main.hpp"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder1;
ESP32Encoder encoder2;

//variable globale
float kp=0, kd=0, offsetDZ1=585.5, offsetDZ2 = 601.0, gyroZ, angleAcc;
static int32_t lastIncrement1 = 0, lastIncrement2 = 0;

//mpu6050
static float angle = 0.0;
float ax, ay, gz;
char FlagCalcul = 0;
float entrerFiltre, anglefiltrer = 0;
// coefficient du filtre
float Te = 5;    // période d'échantillonage en ms
float Tau = 200; // constante de temps du filtre en ms
float A, B;

// Paramètres : Tolérance de 1.5 à 2 degrés
float angletolerer = 0.0;

unsigned long lastVitesseTime;

Adafruit_MPU6050 mpu;

void confGPIO(void){
    //configuration nécessaire
    Serial.begin(115200);
    Wire.begin(21, 22); //initialisation du bus I2C
    // Configurer les broches en I/O
    pinMode(LED, OUTPUT);
    pinMode(VBAT_pin, INPUT);
    pinMode(M1A, INPUT);
    pinMode(M1B, INPUT);
    pinMode(M2A, INPUT);
    pinMode(M2B, INPUT);
    //init moteur
    ledcSetup(CanalM1P, 20000, 10);
    ledcSetup(CanalM1N, 20000, 10);
    ledcSetup(CanalM2P, 20000, 10);
    ledcSetup(CanalM2N, 20000, 10);
    ledcAttachPin(MOT1P, CanalM1P);
    ledcAttachPin(MOT1N, CanalM1N);
    ledcAttachPin(MOT2P, CanalM2P);
    ledcAttachPin(MOT2N, CanalM2N);
}

void calculCoeffFiltre(void){
    A = 1 / (1 + Tau / Te);
    B = Tau / Te * A;
}

void initGyro() {
    confGPIO();
    //initialisation du MPU6050
    while (!Serial) delay(10);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while(1) {
            delay(10);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ay = a.acceleration.y;
    ax = a.acceleration.x;
    angle = atan2(ay, ax) * 180.0 / PI; //calcul de l'angle grace a la foce exercer sur l'axe Y et Z  

    // calcul coeff filtre
    calculCoeffFiltre();

    //init encoder
    encoder1.attachFullQuad(M1A, M1B);
    encoder2.attachFullQuad(M2A, M2B);
    encoder1.setCount(0);
    encoder2.setCount(0);

}

float getAngle() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ay = a.acceleration.y;
    ax = a.acceleration.x;
    gz = g.gyro.z;


    // Utilisation du Gyroscope Y
    gyroZ = -(gz * Tau / 1000.0) * 180.0 / PI; // ConentrerFiltrertir en degrés pour la période d'échantillonnage

    // Calcul de l'angle via l'Accéléromètre
    angleAcc = atan2(ay, ax) * 180.0 / PI;
    entrerFiltre = gyroZ + angleAcc;
    anglefiltrer = A * entrerFiltre + B * anglefiltrer;

    return anglefiltrer;
}

void deplacement(bool mode, signed int vitesse1, signed int vitesse2){
    if (!mode) // controle roue commun
    { 
        if (vitesse1 >  400)
        {
            vitesse1 = 400;
        }
        if (vitesse1 <  -400)
        {
            vitesse1 = -400;
        }
        
        if (vitesse1 >= 0)
        {
            ledcWrite(CanalM1P, offsetDZ1+vitesse1);
            ledcWrite(CanalM1N, 0);
            ledcWrite(CanalM2N, offsetDZ2+vitesse1);
            ledcWrite(CanalM2P, 0);
        }else{
            ledcWrite(CanalM1N, offsetDZ1+abs(vitesse1));
            ledcWrite(CanalM1P, 0);
            ledcWrite(CanalM2P, offsetDZ2+abs(vitesse1));
            ledcWrite(CanalM2N, 0);
        } 
    }
    else{ //controle roue distincte !! A modifier
        if (vitesse1 >= 0)
        {
            ledcWrite(CanalM1P, offsetDZ1+vitesse1);
            digitalWrite(MOT1N, LOW);
        }
        else{
            ledcWrite(CanalM1N, offsetDZ1+abs(vitesse1));
            digitalWrite(MOT1P, LOW);
        }
        if (vitesse2 >= 0)
        {
            ledcWrite(CanalM2N, offsetDZ2+vitesse2); // Attacher MOT1P au canal 0
            digitalWrite(MOT2P, LOW);
        }
        else{
            ledcWrite(CanalM2P, offsetDZ2+abs(vitesse2)); // Attacher MOT1P au canal 0
            digitalWrite(MOT2N, LOW);
        } 
    }  
}

float getVitesse(void){
    float vitesse1, vitesse2;
    static int32_t increment1, increment2;
    // Récupération des incréments depuis les encodeurs
    increment1 = encoder1.getCount() / 2;
    increment2 = encoder2.getCount() / 2;
    vitesse1 = (((float)(increment1 - lastIncrement1) / PPR) * (rayonRoue * 2 * PI)) / 0.005; // m/s
    vitesse2 = (((float)(increment2 - lastIncrement2) / PPR) * (rayonRoue * 2 * PI)) / 0.005; // m/s;
    lastIncrement1 = increment1;
    lastIncrement2 = increment2;

    return (vitesse1 + vitesse2) / 2.0; // Retourne la vitesse gyropode
}


void reception(char ch)
{
  //les chaines de caractères sont parcouru aentrerFiltrec l'indice
  static String chaine = ""; //static 
  String commande;
  String valeur;
  float param;
  int index, length;

  if ((ch == 13) or (ch == 10)) //si on lis CR ou LF donc retour a la ligne (saisie terminé)
  {
    index = chaine.indexOf(' '); //chercher l'espace vide
    length = chaine.length(); 
    if (index == -1) //aucun espace trouvé
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      //commande 0 à l'espace et valeur de l'espace+1 à la fin
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Kp"){
      kp = valeur.toFloat();
    }
    if (commande == "Kd"){
      kd = valeur.toFloat();
    }

    if (commande == "un"){
      offsetDZ1 = valeur.toFloat();
    }
    if (commande == "dx"){
      offsetDZ2 = valeur.toFloat();
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      calculCoeffFiltre();
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
      calculCoeffFiltre();
    }

    chaine = ""; //remetttre la chaine a 0 (écoute)
  }
  else
  {
    chaine = chaine + ch; //stocker caractère
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read()); //chaque caractère est lu et traiter dans reception
  }
}
