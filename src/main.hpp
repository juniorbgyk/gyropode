#ifndef MAIN_HPP
#define MAIN_HPP

//OUTPUT 
#define MOT1P       26
#define MOT1N       25
#define MOT2P       33
#define MOT2N       32
#define LED         16

//INPUT
#define M1A       18
#define M1B       19
#define M2A       17
#define M2B       05
#define VBAT_pin 04

//i2c module
#define SDA_sc            21
#define SCL_sc            22

//constante
#define rayonRoue 0.0325 //en mètre
#define PPR 823 //Pulses Per Revolution, nombre de ticks par tour de roue
#define CanalM1P 0
#define CanalM1N 1
#define CanalM2P 2
#define CanalM2N 3

// Digital IO access macro
#define M1A_VAL    (digitalRead(M1A))
#define M1B_VAL    (digitalRead(M1B))
#define M2A_VAL    (digitalRead(M2A))
#define M2B_VAL    (digitalRead(M2B))
#define LED_OFF (digitalWrite(LED,LOW))
#define LED_ON (digitalWrite(LED,HIGH))
#define VBAT_VAL ((analogRead(VBAT_pin)*9.6)/4095.0)

#endif