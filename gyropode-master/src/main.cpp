#include <Arduino.h>
#include "function.h"
#include <Wire.h>

static float angle, anglecible = 0.0;

extern float kp, kd, entrerFiltre, offsetDZ1, offsetDZ2, angleAcc, gyroZ;
extern float ax, ay, gz;

volatile bool flag = false;

float ec, err, last_err = 0;

extern float Te;    // période d'échantillonage en ms
extern float Tau; // constante de temps du filtre en ms

void moveTask(void *parametres) //tahe asservissement et deplacement
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    angle = getAngle();
    err = anglecible - angle; // angle cible est 0
    ec = (kp * err) - (kd * (-gz * 180.0 / PI));
    deplacement(0,-ec);
    flag = true;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void ihmTask(void *parametres)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    serialEvent();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup() {
  initGyro();
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

  // vTaskDelete(NULL);  //a la fin du setup pour pas executer le loop
}

void loop() {
  if (flag)
  {
    Serial.printf("%lf %lf %lf %lf \n", angle, kp, kd, ec);
    flag = false;
  }
  
}