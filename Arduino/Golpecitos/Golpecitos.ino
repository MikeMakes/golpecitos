
#include "Golpecitos.h"

#define vmax 800
#define vcrucero 300

/*
ENABLE(PWM) |      A       |     B        |   STATUS           |
--------------------------------------------------------- |
     LOW    |    ----      |    ----      |MOTOR PARADO        |
     HIGH   |    HIGH      |     LOW      |MOTOR GIRA DELANTE  |
     HIGH   |    LOW       |     HIGH     |MOTOR GIRA AL REVES |
     HIGH   |    HIGH      |     HIGH     |MOTOR PARADO        |
     HIGH   |    LOW       |     LOW      |MOTOR PARADO        |
 */

Golpecitos golpes(7,6);

void setup() {
  //Serial.begin(9600); debug, cuidao que en golpecitos hay serial.println que no se si chocan con el bluetooth
  
  golpes.inicialize();
  golpes.saluda();

}

void loop() {

  golpes.readSonar(1);

  
  // golpes.move(vmax,0);
  // delay(1000);

  // golpes.move(-vmax,0);
  // delay(1000);

  // golpes.move(0,vmax);
  // delay(1000);

  // golpes.move(0,-vmax);
  // delay(1000);

  // golpes.move(vmax,vcrucero);
  // delay(1000);

  // golpes.move(-vmax,-vcrucero);
  // delay(1000);

  // golpes.move(0,0);
  // delay(1000);
}
