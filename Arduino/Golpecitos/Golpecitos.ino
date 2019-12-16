#include "Golpecitos.h"


Golpecitos golpes(6,7,8,9); // (echo izq , trig izq , echo dcha , echo izq)

void setup() {
  
  golpes.inicialize();

  golpes.mRobotMode = 0; // Se fija el modo del robot
}

void loop() {
  //golpes.readBluetooth();
  switch (golpes.mRobotMode){
    case 0:
      golpes.Stop();
      break;
    
    case 5:
      golpes.step();
      break;
    
    case 1://distancia frontal
      golpes.changePID();
      golpes.changeYawPID();
      golpes.stepControl();
      break;

    case 2: //angulo frontal
      golpes.changePID();
      golpes.changeYawPID();
      golpes.stepControl();
      break;

    case 3: //angulo paralelo
      // golpes.changePID();
      golpes.changeYawPID();
      golpes.stepControlParallel();
      break;

    case 4://distancia paralelo
      golpes.changePID();
      // golpes.changeYawPID();
      golpes.stepControlParallel();
      break;

    default:
      break;
  }
  golpes.writeTelemetry();
  golpes.changeState();
  delay(100);
}
