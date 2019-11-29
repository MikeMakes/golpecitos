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

Golpecitos golpes(6,7,8,9); // (echo izq , trig izq , echo dcha , echo izq)

void setup() {
  
  golpes.inicialize();
  golpes.saluda();

  golpes.mRobotMode = 1; // Se fija el modo del robot
}

void loop() {
  golpes.desatado();
  delay(100);
}
