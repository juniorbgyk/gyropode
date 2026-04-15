#include <Arduino.h>
#include "function.h"
#include <Wire.h>
#include <BluetoothSerial.h>

// Variables globales de contrôle utilisées dans la boucle principale
static float angle, vitesse, vitesseCible = 0.0;

// Variables externes déclarées dans function.cpp
extern float kp, kd, v_kd, v_kp, entrerFiltre, offsetDZ1, offsetDZ2, angleAcc, gyroZ, offsetAngle;
extern float ax, ay, gz, vitesse1, vitesse2;

volatile bool flag = false; // drapeau volatile pour synchronisation éventuelle

// Variables de calcul d'asservissement
float ec, err, err_vitessse, ajustVitesse, err_lastVitesse = 0;
int joystickX, joystickY; // valeurs de commande du joystick reçues en Bluetooth

// Paramètres de filtrage externes définis dans function.cpp
extern float Te;    // période d'échantillonnage en ms
extern float Tau;   // constante de temps du filtre en ms

BluetoothSerial SerialBT; // Interface Bluetooth série

void moveTask(void *parametres) // tâche d'asservissement et de déplacement
{
  //Essentielle pouur l'utilisation périodique de la tache
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // Lecture des capteurs et mise à jour de l'état actuel
    angle = getAngle();
    vitesse = getVitesse();

    // Conversion de la commande joystick Y en consigne de vitesse cible
    if (joystickY >= 0) vitesseCible = joystickY * (-0.0041) / 4.0; //ici /4 acr moteur était trop rapide dans ce sens 
    if (joystickY < 0) vitesseCible = joystickY * (-0.0041) / 2.0;

    // Erreur de vitesse cible par rapport à la vitesse mesurée
    err_vitessse = vitesseCible - vitesse;

    // Régulateur PD pour la vitesse
    ajustVitesse = (err_vitessse * v_kp) + (v_kd * (err_vitessse - err_lastVitesse));

    // Limitation de l'action de vitesse pour éviter de trop pousser les moteurs
    if (ajustVitesse > 10) ajustVitesse = 10;
    if (ajustVitesse < -10) ajustVitesse = -10;

    // Calcul de l'erreur d'angle à corriger pour maintenir l'équilibre
    err = (ajustVitesse + offsetAngle) - angle;
    ec = ((kp * err) - (kd * (-gz * 180.0 / PI)));
    err_lastVitesse = err_vitessse;

    // Ordres de déplacement avec compensation de l'axe X du joystick
    if (-ec >= 0)
      deplacement(1, -ec + joystickX, (-ec - 10) - joystickX); //-ec-10 pour compenser le fait que le robot tend vers la gauche
    if (-ec < 0)
      deplacement(1, -ec + joystickX, (-ec + 10) - joystickX);

    // Serial.printf("%f %f %f %f \n", vitesseCible, vitesse, offsetAngle, angle); // Affichage de données pour le debug

    // Attente périodique sur base de Te
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void ihmTask(void *parametres) // tâche d'interface homme-machine
{
  //Essentielle pouur l'utilisation périodique de la tache
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // Lecture des données Bluetooth si une commande est disponible
    if (SerialBT.available()) {
      LED_ON; // allume une LED pour signaler la connexion Bluetooth active
      char commande = SerialBT.read();

      if (commande == 'X') {
        joystickX = SerialBT.parseInt();
      }
      if (commande == 'Y') {
        joystickY = SerialBT.parseInt();
      }
    }

    // Envoi des informations de tension et vitesse au client Bluetooth connecté
    if (SerialBT.hasClient()) {
      SerialBT.printf("*v %.2f", VBAT_VAL); //*v est la clé pour etre comprise par le client Bluetooth comme une donnée utilisable
      SerialBT.printf("*s %.2f", vitesse); //*v est la clé pour etre comprise par le client Bluetooth comme une donnée utilisable
      //les clés sont configurable depuis l'app utilisé (bluetooth electronics)
    }

    // Traitement des commandes série USB
    serialEvent();

    // Attente périodique sur base de Te
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup() {
  // Initialisation du gyroscope, des encodeurs et des moteurs
  initGyro();

  // Démarrage de la communication Bluetooth avec le nom de l'appareil
  SerialBT.begin("malcom");

  // Création de la tâche de mouvement
  xTaskCreatePinnedToCore(
    moveTask,       // fonction de la tâche
    "MoveTask",    // nom de la tâche
    10000,          // taille de pile en octets
    NULL,           // paramètre passé à la tâche
    3,              // priorité
    NULL,           // handle de tâche non utilisé
    1               // exécution sur le core 1
  );

  // Création de la tâche d'interface homme-machine
  xTaskCreatePinnedToCore(
    ihmTask,
    "IhmTask",
    10000,
    NULL,
    2,
    NULL,
    1 // exécution sur le core 1
  );

  vTaskDelete(NULL); //a la fin du setup pour supprimer la tâche setup après execution pour empêcher l'exécution de loop()
}

void loop() {
  // Vide : les tâches FreeRTOS gèrent le programme
}