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
//i2c module
#define SDA_sc            21
#define SCL_sc            22

//constante

// Digital IO access macro
#define M1A_VAL    (digitalRead(M1A))
#define M1B_VAL    (digitalRead(M1B))
#define M2A_VAL    (digitalRead(M2A))
#define M2B_VAL    (digitalRead(M2B))
#define LED_OFF (digitalWrite(LED,LOW))
#define LED_ON (digitalWrite(LED,HIGH))

#endif