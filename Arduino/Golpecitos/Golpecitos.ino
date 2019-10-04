
#include "Golpecitos.h"

/*
ENABLE(PWM) |      A       |     B        |   STATUS           |
--------------------------------------------------------- |
     LOW    |    ----      |    ----      |MOTOR PARADO        |
     HIGH   |    HIGH      |     LOW      |MOTOR GIRA DELANTE  |
     HIGH   |    LOW       |     HIGH     |MOTOR GIRA AL REVES |
     HIGH   |    HIGH      |     HIGH     |MOTOR PARADO        |
     HIGH   |    LOW       |     LOW      |MOTOR PARADO        |
 */

/* Define the Motors pin numbers */
#define mL_en 2
#define mR_en 11
// Motors directions
#define mL_a 3
#define mL_b 4
#define mR_a 13
#define mR_b 12

// Define robot distances
#define r 3.15 // Radio de la rueda
#define k 6.425 //Distancia del centro a la rueda

// Global variables
float w[2]; // Wheels velocities {left, right}

void write_pwm(int enable,int pwm, int dir1, int dir2){ //function to write pwm 2 motors & manage directions

    if(pwm > 0){
      if (pwm>255) pwm=255;
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        analogWrite(enable, pwm);
    }
    else if(pwm < 0){
      if (pwm<-255) pwm=-255;
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(enable, abs(pwm));
    }
    else{
        digitalWrite(enable, LOW);
    }
}

void move(){  // escribe la velocidad w, generalmente calculada
  write_pwm(mR_en,w[1], mR_a,mR_b);
  write_pwm(mL_en, w[0], mL_a, mL_b);
}

void vel(float lin, float ang){ //modelo cinematico
 float acoplamiento = ang / lin;
 w[0]=(lin*(1-k*acoplamiento))/r;
 w[1]=(lin*(1+k*acoplamiento))/r;  
}


void setup() {
    /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT); pinMode(mR_en, OUTPUT);

  Golpecitos golpes;
  golpes.saluda();



}

void loop() {
  vel(1000,0);  // un pasito palante
  move();
  delay(2000);
  vel(-1000,0); // un pasito pa tras
  move();
  delay(2000);
  vel(0,-1000);
  delay(2000); /dch o izq 
  
}
