#include <Arduino.h>
#include "function.h"
#include <Wire.h>
#include <MPU6050.h>

void moveTask(void *parametres) //tahe asservissement et deplacement
{
  while (1)
  {
    vTaskDelay(1000); //La tache se met en pause pendant x temps et le processeur exécute d'autres tâches en attendant 
  }
}

void angleTask(void *parametres)
{
  while (1)
  { 
    vTaskDelay(1000);
  }
}

void vitesseTask(void *parametres)
{
  while (1)
  { 
    vTaskDelay(1000);
  }
}

void ihmTask(void *parametres)
{
  while (1)
  {
    serialEvent();
    vTaskDelay(1000);
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
    6,             // Priorité
    NULL,      // Handle (optionnel)
    1
  );

  xTaskCreatePinnedToCore( 
    angleTask,       
    "AngleTask",
    10000,        
    NULL,          
    5,             
    NULL,
    1         
  );

  xTaskCreatePinnedToCore( 
    vitesseTask,       
    "VitesseTask",     
    10000,       
    NULL,          
    4,             
    NULL,
    1        
  );

  xTaskCreatePinnedToCore( 
    ihmTask,    
    "IhmTask",    
    10000,         
    NULL,          
    3,            
    NULL,
    0 // Core 0 pour pour eviter latence communication sans fil impacte gyro et asservissement         
  );

  vTaskDelete(NULL);  //a la fin du setup pour pas executer le loop
}

void loop() {
}