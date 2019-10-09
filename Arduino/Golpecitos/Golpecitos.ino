
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

Golpecitos golpes(6,7,44,45);

void setup() {
  //Serial.begin(9600); debug, cuidao que en golpecitos hay serial.println que no se si chocan con el bluetooth
  
  golpes.inicialize();
  golpes.saluda();

}

void loop() {

  // float distanciaSonar1 = golpes.readSonar(1);
  // Serial.print(distanciaSonar1);
  // Serial.print("cm");
  // Serial.println();


  //golpes.readBluetooth();
  golpes.step();


  delay(1000);
}
