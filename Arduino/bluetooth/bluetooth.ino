const int led = 13;
 
int option;
 
void setup(){
  Serial.begin(9600);
  pinMode(led, OUTPUT); 
   Serial1.begin(38400);
}
 
void loop(){
  //si existe información pendiente
  if (Serial1.available()>0){
    //leeemos la opcion
    char option = Serial1.read();
     Serial.write(option);
    
    }
  }
