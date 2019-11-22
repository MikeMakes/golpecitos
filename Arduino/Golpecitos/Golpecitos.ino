
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

  golpes.mRobotMode = 0; // Se fija el modo del robot
}

void loop() {

  float distanciaSonar1 = golpes.readSonar(1);
  float distanciaSonar2 = golpes.readSonar(0);
  Serial.print(String("Distancia sonar izquierdo: ")+ distanciaSonar1 + String(" cm\n") + String("Distancia sonar derecho: ")+ distanciaSonar1 + String(" cm"));
  Serial.println();

  golpes.runControl();
  golpes.stepControl();
  golpes.writeTelemetry();


  delay(1000);
}
