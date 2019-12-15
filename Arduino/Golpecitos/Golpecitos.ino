#include "Golpecitos.h"


Golpecitos golpes(6,7,8,9); // (echo izq , trig izq , echo dcha , echo izq)

void setup() {
  
  golpes.inicialize();

  golpes.mRobotMode = 0; // Se fija el modo del robot
}

void loop() {

  golpes.changeYawPID(); //Checks for a change in pid  from phone
  golpes.stepControl(); //Apply control
  golpes.writeTelemetry();  //Log out telemetry by bluetooth : incT [ms] , distIzq [cm] , distDcha [cm] , ref [cm] , modo [int] , velPWMizq [int] , velPWMdcha [int]

  delay(100);
}
