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
unsigned long lastTime, lastVitesseTime;

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
    }
    calibrageGyroscope(); //premet davoir une meilleur précision du gyroscope en compensant les biais de mesure
    // --- Permet de partir de l'angle réel pour l'asservissement ---
    mpu.getAcceleration(&ax, &ay, &az);
    angle = atan2(ay, ax) * 180.0 / PI; //calcul de l'angle grace a la foce exercer sur l'axe Y et Z   
    lastTime = millis(); //récupère dernier valeur compter en ms (quand l'esp est alimenté il commence a compter le temps)
    lastVitesseTime = millis();
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

void deplacement(bool mode, unsigned int vitesse1, unsigned int vitesse2){
    if (!mode) // controle roue commun
    {
        if (vitesse1 >= 0)
        {
            ledcWrite(CanalM1P, offsetDZ+vitesse1);
            digitalWrite(MOT1N, LOW);
            ledcWrite(CanalM2P, offsetDZ+vitesse1);
            digitalWrite(MOT2N, LOW);
        }
        else{
            ledcWrite(CanalM1N, offsetDZ+vitesse1);
            digitalWrite(MOT1P, LOW);
            ledcWrite(CanalM2N, offsetDZ+vitesse1);
            digitalWrite(MOT2P, LOW);
        } 
    }
    else{ //controle roue distincte
        if (vitesse1 >= 0)
        {
            ledcWrite(CanalM1P, offsetDZ+vitesse1);
            digitalWrite(MOT1N, LOW);
        }
        else{
            ledcWrite(CanalM1N, offsetDZ+vitesse1);
            digitalWrite(MOT1P, LOW);
        }
        if (vitesse2 >= 0)
        {
            ledcWrite(CanalM2P, offsetDZ+vitesse1); // Attacher MOT1P au canal 0
            digitalWrite(MOT2N, LOW);
        }
        else{
            ledcWrite(CanalM2N, offsetDZ+vitesse1); // Attacher MOT1P au canal 0
            digitalWrite(MOT2P, LOW);
        } 
    }  
}

float getVitesse(void){
    /*
    commenter et lui envoyer par mail guinand
    Demander si je dois:
        - essayer davoir 2 dt différent
        - incrément tjr = 1 si = 0 alors est-ce que je gère bien
        -que faire en cas d'avancement reculement rapide donc vitesse fausse
        - finalement code global est vrai ?
    */
    static int lastM1, lastM2, increment1=0, increment2=0;
    int currentM1 = (M1A_VAL ^ M1B_VAL), currentM2 = (M2A_VAL ^ M2B_VAL);
    unsigned long currentVitesseTime, dt;
    float vitesse1, vitesse2;
    
    // --- Détection front montant ---
    if (!lastM1 && currentM1) increment1++;
    if (!lastM2 && currentM2) increment2++;
    lastM1 = currentM1;
    lastM2 = currentM2;
    
    // calcul de la vitesse
    currentVitesseTime = millis();
    dt = currentVitesseTime - lastVitesseTime;
    if (dt == 0) dt = 1;  // Éviter division par zéro
    if (dt < 10) return 0.0;  // limite mesure fausse quand gyro

    vitesse1 = (((float)increment1/PPR)*(rayonRoue*2*PI))/(dt/1000.0); //vitesse en m/s
    vitesse2 = (((float)increment2/PPR)*(rayonRoue*2*PI))/(dt/1000.0); //vitesse en m/s
    lastVitesseTime = currentVitesseTime;
    increment1 = 0;
    increment2 = 0;
    return (vitesse1 + vitesse2) / 2.0; // Retourne la vitesse gyropode
}


void reception(char ch)
{
  //les chaines de caractères sont parcouru avec l'indice
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
    // afficher les paramètres
    Serial.printf("commande '%s' \r\n", commande);
    Serial.printf("valeur '%s' \r\n", valeur);
 

    if (commande == "BF")
    {
      //ce que je veux
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