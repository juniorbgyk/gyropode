#include <Arduino.h>
#include "function.h"
#include <Wire.h>
#include <BluetoothSerial.h>

//kp=58.1, Kd= 2.28, vdd=7.2
static float angle, vitesse, vitesseCible = 0.0;

extern float kp, kd, v_kd, v_kp, entrerFiltre, offsetDZ1, offsetDZ2, angleAcc, gyroZ, offsetAngle;
extern float ax, ay, gz, vitesse1, vitesse2;

volatile bool flag = false;

float ec, err, err_vitessse, ajustVitesse, err_lastVitesse = 0;
int joystickX, joystickY;

extern float Te;    // période d'échantillonage en ms
extern float Tau; // constante de temps du filtre en ms

BluetoothSerial SerialBT;

void moveTask(void *parametres) //tahe asservissement et deplacement
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    //Corrigez VKP parce qu'il se rattrape trop violemment sinon asservisement ok et deplacement en Y ok reste aussi deplacement en X
    angle = getAngle();
    vitesse = getVitesse();
    if (joystickY >= 0) vitesseCible = joystickY*(-0.0041)/4.0;
    if (joystickY < 0) vitesseCible = joystickY*(-0.0041)/2.0;
    err_vitessse = vitesseCible - vitesse;
    ajustVitesse = (err_vitessse * v_kp) + (v_kd*(err_vitessse-err_lastVitesse));
    if (ajustVitesse > 10) ajustVitesse = 10;
    if (ajustVitesse < -10) ajustVitesse = -10;
    err = (ajustVitesse+offsetAngle) - angle; 
    ec = ((kp * err) - (kd * (-gz * 180.0 / PI)));
    err_lastVitesse = err_vitessse;
    if (-ec >= 0) deplacement(1, -ec+(joystickX), (-ec-10)-(joystickX));
    if (-ec < 0) deplacement(1, -ec+(joystickX), (-ec+10)-(joystickX));
    Serial.printf("%f %f %f %f \n", vitesseCible, vitesse, offsetAngle, angle);
     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void ihmTask(void *parametres)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // Serial.println("Vbat:");
    // Serial.println(VBAT_VAL);
    if (SerialBT.available()) {
      LED_ON;
      char commande = SerialBT.read();

      if (commande == 'X') {
        joystickX = SerialBT.parseInt();
      }
      if (commande == 'Y') {
        joystickY = SerialBT.parseInt();
      }
    }

    if (SerialBT.hasClient()) {
      SerialBT.printf("*v %.2f", VBAT_VAL);
      SerialBT.printf("*s %.2f", vitesse);
    }
    serialEvent();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup() {
  initGyro();
  SerialBT.begin("malcom");
  //demander comment estimer taille pile pour chaque tache
  xTaskCreatePinnedToCore(
    moveTask,       // Fonction de la tâche
    "MoveTask",     // Nom
    10000,          // Taille de pile
    NULL,          // Paramètre
    3,             // Priorité
    NULL,      // Handle (optionnel)
    1
  );

  xTaskCreatePinnedToCore( 
    ihmTask,    
    "IhmTask",    
    10000,         
    NULL,          
    2,            
    NULL,
    1 // Core 0 pour pour eviter latence communication sans fil impacte gyro et asservissement         
  );
  vTaskDelete(NULL);  //a la fin du setup pour pas executer le loop
}

void loop() {
}