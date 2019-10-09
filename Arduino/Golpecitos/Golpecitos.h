#ifndef Golpecitos_h
#define Golpecitos_h

#include <Arduino.h>

#define r 3.15 // Radio de las ruedas
#define eje 6.425 // Longitud del eje / 2

/*
ENABLE(PWM) |      A       |     B        |   STATUS           |
--------------------------------------------------------- |
     LOW    |    ----      |    ----      |MOTOR PARADO        |
     HIGH   |    HIGH      |     LOW      |MOTOR GIRA DELANTE  |
     HIGH   |    LOW       |     HIGH     |MOTOR GIRA AL REVES |
     HIGH   |    HIGH      |     HIGH     |MOTOR PARADO        |
     HIGH   |    LOW       |     LOW      |MOTOR PARADO        |
 */

class Golpecitos{

    public:
      Golpecitos(int _pinEcho,int _pinTrig);
      ~Golpecitos();

      void inicialize();
      void saluda();
      void cinematica(float lin, float ang) ;
      void move(float _lin, float _ang);
      void write_pwm(int _enable,int _pwm, int _dir1, int _dir2);


      void readSonar(int _sonarNum);

    public:
      float mSpeed[2]; // Velocidad de las ruedas: 0-izq y 1-dch
      float mDistSonar;

   private:
    int mPinTrig = 7;
    int mPinEcho = 6;

    // Define the Motors pin numbers (velocity - pwm) 
    int mL_en= 2;
    int mR_en= 11;
    // Motors directions
    int mL_a =3;
    int mL_b =4;
    int mR_a =13;
    int mR_b =12;
 
    const float mVelSon = 34000.0;

   private:

    void iniciarTrigger();

};


#endif
