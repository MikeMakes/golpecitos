
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

Golpecitos golpes(6,7,8,9);

void setup() {
  //Serial.begin(9600); debug, cuidao que en golpecitos hay serial.println que no se si chocan con el bluetooth
  
  golpes.inicialize();
  golpes.saluda();

}

void loop() {

 float distanciaSonar1 = golpes.readSonar(1);
 float distanciaSonar2 = golpes.readSonar(0);

// Serial.print(distanciaSonar1);
// Serial.print("cm");
// Serial.print("       ");
 //Serial.print(distanciaSonar2);
 //Serial.print("cm");
// Serial.println();
 //Serial.print("       ");
 


//golpes.writeTelemetry();
golpes.runControl();


  delay(1000);
}
