
 int option;

void setup() {
   Serial.begin(9600);
  pinMode(led, OUTPUT); 
   Serial1.begin(38400);

}

void loop() {
  //si existe información pendiente
  if (Serial1.available()>0){
    char option = Serial1.read();

 switch (option){

  case 'a':       //Avanza
        Serial.write("Avanza");
        break;
  case 'r':      //Retrocede
        Serial.write("Retrocede");
        break;
  case '1':      //Rotar sentido horario
        Serial.write("Rotar sentido horario");
        break;
  case '2':     //Rotar en sentido antihorario
        Serial.write("Rotar sentido antihorario");
        break;
  case  'i':    //Girar Izquierda (hacia adelante y atrás)
        Serial.write("Girar Izquierda (hacia adelante y atrás)");
        break;
  case  'd':     //Girar Derecha (hacia adelante y atrás)
        Serial.write("Girar Derecha (hacia adelante y atrás)");
        break;
         
  }
 }

}
