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

// Define the Motors pin numbers (velocity - pwm) 
#define mL_en 2
#define mR_en 11
// Motors directions
#define mL_a 3
#define mL_b 4
#define mR_a 13
#define mR_b 12

class Golpecitos{

    // Public variables (Puede acceder cualquiera)
    public:
      Golpecitos();
      ~Golpecitos();
      float mSpeed[2]; // Velocidad de las ruedas: 0-izq y 1-dch
    // Public functions (Puede acceder cualquiera)
    public:
		  void saluda();
      float* cinematica(float lin, float ang) ;
      void move(float _lin, float _ang);
      void write_pwm(int _enable,int _pwm, int _dir1, int _dir2);

   // Private variables (solo puede acceder la propia clase)
   private:

   // Private functions (solo puede acceder la propia clase)
   private:


};


#endif
