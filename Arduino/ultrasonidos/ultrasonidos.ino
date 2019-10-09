
 
// Configuramos los pines del sensor Trigger y Echo
const int PinTrigD = 9;
const int PinEchoD = 8;
const int PinTrigI = 7;
const int PinEchoI = 6;
 
// Constante velocidad sonido en cm/s
const float VelSon = 34000.0;

float distancia;
float distanciaDER;
float distanciaIZQ;

void setup() 
{//Bluetooth
Serial1.begin(38400);
  
  // Iniciamos el monitor serie para mostrar el resultado
  Serial.begin(9600);
  // Ponemos el pin Trig en modo salida
  pinMode(PinTrigI, OUTPUT);
  pinMode(PinTrigD, OUTPUT);
  // Ponemos el pin Echo en modo entrada
  pinMode(PinEchoI, INPUT);
  pinMode(PinEchoD, INPUT);


 Serial.print("distanciaIZQ");
  Serial.print("          ");
  Serial.print("distanciaDER");
  Serial.print("cm");
  Serial.println();
}

float Sonar(int PinTrig,int PinEcho){
  iniciarTrigger(PinTrig);

  // La función pulseIn obtiene el tiempo que tarda en cambiar entre estados, en este caso a HIGH
  unsigned long tiempo = pulseIn(PinEcho, HIGH);
  
    // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001
  distancia = tiempo * 0.000001 * VelSon / 2.0;

  return distancia;
}
  
void loop()
{  
distanciaDER= Sonar(PinTrigD,PinEchoD);
distanciaIZQ= Sonar(PinTrigI,PinEchoI);

  Serial.print(distanciaIZQ);
  Serial.print("          ");
  Serial.print(distanciaDER);
  Serial.print("cm");
  Serial.println();
  delay(1000);
}

// Método que inicia la secuencia del Trigger para comenzar a medir
void iniciarTrigger(int PinTrig)
{
  // Ponemos el Triiger en estado bajo y esperamos 2 ms
  digitalWrite(PinTrig, LOW);
  delayMicroseconds(2);
  
  // Ponemos el pin Trigger a estado alto y esperamos 10 ms
  digitalWrite(PinTrig, HIGH);
  delayMicroseconds(10);
  
  // Comenzamos poniendo el pin Trigger en estado bajo
  digitalWrite(PinTrig, LOW);

}
