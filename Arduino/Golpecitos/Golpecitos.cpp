
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha) {
	Serial.println("Se ha llamado al constructor de la clase");

  mPinEcho[0] = _pinEchoIzq;
  mPinTrig[0] = _pinTrigIzq;

  mPinEcho[1] = _pinEchoDcha;
  mPinTrig[1] = _pinTrigDcha;

}

// ----------- Destructor de la clase -----------
Golpecitos::~Golpecitos(){}

//  ----------- Metodos(funciones) publicos -----------
void Golpecitos::saluda(){
	Serial.println("hola jeje");
	return ;
}

//----------------------------------------------------------------------------------
void Golpecitos::inicialize(){
    // Iniciamos el monitor serie para mostrar el resultado
  Serial.begin(9600);
  // Ponemos el pin Trig en modo salida
  pinMode(mPinTrig[0], OUTPUT);
  pinMode(mPinTrig[1], OUTPUT);
  // Ponemos el pin Echo en modo entrada
  pinMode(mPinEcho[0], INPUT);
  pinMode(mPinEcho[1], INPUT);

  /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT);  pinMode(mR_en, OUTPUT);

  // Configure bluetooth seral
  Serial1.begin(38400);


  // Configure controller pointer
  mPid = new PID(-100.0, 0.0 , 10.0 ,-803.0,803.0); // Kp , Ki , Kd , minSat , maxSat
  mPid->reference(30.0);

  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::write_pwm(int _enable,int _pwm, int _dir1, int _dir2){ //function to write _pwm 2 motors & manage directions
    if(_pwm > 0){

      if (_pwm>255) _pwm=255;
      
      digitalWrite(_dir1, HIGH);
      digitalWrite(_dir2, LOW);
      analogWrite(_enable, _pwm);

    }	else if(_pwm < 0){

      if (_pwm<-255) _pwm=-255;
      
      digitalWrite(_dir1, LOW);
      digitalWrite(_dir2, HIGH);
      analogWrite(_enable, abs(_pwm));

    }else{
        digitalWrite(_enable, LOW);
    }

    return;
}


//----------------------------------------------------------------------------------
void Golpecitos::cinematica(float _lin, float _ang){

	mSpeed[0]=(_lin - mDistWheels * _ang) /mWheelRadius;
	mSpeed[1]=(_lin + mDistWheels * _ang) /mWheelRadius;

	return;
}

//----------------------------------------------------------------------------------
void Golpecitos::move(float _lin, float _ang){ //input _vel[0]=velocidad_rueda_izq _vel[1]=velocidad_rueda_dch
	cinematica(_lin,_ang);
  
	write_pwm(mL_en, mSpeed[0], mL_a, mL_b);
	write_pwm(mR_en, mSpeed[1], mR_a, mR_b);

  return;
}

//----------------------------------------------------------------------------------
float Golpecitos::readSonar(int _sonarNum){  
  iniciarTrigger(mPinTrig[_sonarNum]);
  // La función pulseIn obtiene el tiempo que tarda en cambiar entre estados, en este caso a HIGH
  unsigned long tiempo = pulseIn(mPinEcho[_sonarNum], HIGH);
    // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001
  mDistSonar[_sonarNum] = tiempo * 0.000001 * mVelSon / 2.0;

  return mDistSonar[_sonarNum];
}


//----------------------------------------------------------------------------------
void Golpecitos::iniciarTrigger(int _pinTrig){
  // Ponemos el Triiger en estado bajo y esperamos 2 ms
  digitalWrite(_pinTrig, LOW);
  delayMicroseconds(2);
  
  // Ponemos el pin Trigger a estado alto y esperamos 10 ms
  digitalWrite(_pinTrig, HIGH);
  delayMicroseconds(10);
  
  // Comenzamos poniendo el pin Trigger en estado bajo
  digitalWrite(_pinTrig, LOW);
}

//----------------------------------------------------------------------------------
char Golpecitos::readBluetooth(){
  if (Serial1.available()>0){
    //leeemos la opcion
    mBluetoothData = Serial1.read();
    // Serial.write(mBluetoothData);
    }

  return mBluetoothData;
}

//----------------------------------------------------------------------------------
void Golpecitos::step(){

  char value;
  value = readBluetooth();

  // APARTADO 1
  switch (value){
    case '1': // Avanza linea recta
      move(mVelCrucero , 0.0);
      break;

    case '2': // rotar sentido horario
      move(0.0 , mVelCrucero );
      break;

    case '3':  // retrocede
      move(-mVelCrucero , 0.0);
      break;

    case '4':  // rotar sentido antihorario
      move(mVelCrucero , 0.0);
      break;

    case '5': // girar izquierda
      move(mVelMax - 200.0 , mVelCrucero);
      break;

    case '6':  // girar derecha
      move(mVelCrucero , mVelMax - 200.0);
      break;

    default:
      move(0.0 , 0.0);
      break;
  }
  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::runControl(){
  //Identificar kd,kp,o ki
  //char char1;
  //String gain;
  int i=0;
      if(readBluetooth()=='I'){
        Serial.print("KI=");
        while(i<=2 ){
          i++;
          Serial.print(readBluetooth());
        }
        i=0;
        Serial.println();
      }
      else if(readBluetooth()=='P'){
        Serial.print("KP=");
        while(i<=2 ){
          i++;
          Serial.print(readBluetooth());
        }
        i=0;
        Serial.println();
      }
      else if(readBluetooth()=='D'){
        Serial.print("KD=");
        while(i<=2 ){
          i++;
          Serial.print(readBluetooth());
        }
        i=0;
        Serial.println();
      }
      
    
  
    //Serial.print("antes  ");
    //Serial.print(mPid->mKp);
    //float newK = -20.0;
    //mPid->mKp = newK;//mPid->kp(float gain);
    //Serial.print("despues  ");
    //Serial.print(mPid->mKp);
    //Serial.print(gain);
}
  

//----------------------------------------------------------------------------------
void Golpecitos::stepControl(){

  // Feed PIDs
  float currentTime = millis();
  double incT = double(currentTime - mLastTime);

  readSonar(0); // 0 es izquierda y 1 es derecha
  readSonar(1);
  float distanciaMedia = ( mDistSonar[0]+mDistSonar[1] ) / 2.0;
  float outPID = mPid->update( distanciaMedia , incT); // entrada -> medida ; salida -> (?)

  // Aqui se deberia actuar con la salida del control
  move(outPID,0.0); // (linear , angular)

  mLastTime = millis();
}

//----------------------------------------------------------------------------------
void Golpecitos::writeTelemetry(){
  // Definir string para mandar Aqui
  // log -> incT [ms] , distIzq [cm] , distDcha [cm] , ref [cm] , modo [int] , velPWMizq [int] , velPWMdcha [int]
  String log = String(mLastTime) + " " + String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPid->mKp) + " " + String(mPid->mKi) + " " + String(mPid->mKd)
              + " " + String(mPid->reference()) + " " + String(mRobotMode) + " " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";
  
  Serial1.print(log);

  return;
}
