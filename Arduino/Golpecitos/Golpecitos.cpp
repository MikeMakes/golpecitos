
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha) {
	// Serial.println("Se ha llamado al constructor de la clase");
  mPinEcho[0] = _pinEchoIzq;
  mPinTrig[0] = _pinTrigIzq;

  mPinEcho[1] = _pinEchoDcha;
  mPinTrig[1] = _pinTrigDcha;
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
  mPid    = new PID(-75.0, 0.5 , 0.0 ,-803.0,803.0 , 50, -50); //P -100 funciona: P-75,I-0.5 funciona:
  mPid->reference(30.0);

  mPidAng = new PID(50.0, 0.1 , 0.0 , -803.0,803.0 , 50, -50);//SATURAR EL PWM ANGULAR PARA QUE NO SE VUELVA LOCO
  mPidAng->reference(0.0);

  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::write_pwm(int _enable,int _pwm, int _dir1, int _dir2){ //function to write _pwm 2 motors & manage directions
  if(_pwm > 0){
    // saturate signal
    if (_pwm>255) _pwm=255; 

    digitalWrite(_dir1, HIGH);
    digitalWrite(_dir2, LOW);
    analogWrite(_enable, _pwm);
  }else if(_pwm < 0){
    // saturate signal
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
	write_pwm(mR_en, mSpeed[1], mR_a,mR_b);

  return;
}

void Golpecitos::Stop(){
  move(0,0);
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
  
  return;
}

//----------------------------------------------------------------------------------
char Golpecitos::readBluetooth(){
  if (Serial1.available()>0){
    do {
    mBluetoothData = Serial1.read();
    } while (mBluetoothData == '*');
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
      move(mVelMax , 0.0);
      break;

    case '2': // rotar sentido horario
      move(0.0 , -mVelCrucero );
      break;

    case '3':  // retrocede
      move(-mVelMax , 0.0);
      break;

    case '4':  // rotar sentido antihorario
      move(0.0, mVelCrucero);
      break;

    case '8': // girar izquierda
      move(mVelMax - 200.0 , mVelCrucero);
      break;

    case '5':  // girar derecha
      move(mVelCrucero , mVelMax - 200.0);
      break;

    case 'M':
      value = readBluetooth(); //pasamos a int
      mRobotMode =  value - '0';
      break;

    default:
      move(0.0 , 0.0);
      break;
  }
  return;
}

void Golpecitos::changeState(){
  char charReceived;
  charReceived = readBluetooth();

  if(charReceived == 'M'){
    charReceived = readBluetooth(); //pasamos a int
    mRobotMode =  charReceived - '0';
  }
    
}

//----------------------------------------------------------------------------------
void Golpecitos::changePID(){  // Checks for a change request of P,I,D and Ref from bluetooth
  char charReceived, param;
  String number;

  charReceived=readBluetooth();
  if(charReceived == 'P' || (charReceived == 'I') || (charReceived == 'D') || (charReceived == 'R')){
    param = charReceived;
    
    charReceived=readBluetooth();
    while(charReceived!='*') {
      number += charReceived;
      charReceived=readBluetooth();
    }

  if (param == 'P') mPid->mKp = number.toFloat();
  if (param == 'I') mPid->mKi = number.toFloat();
  if (param == 'D') mPid->mKd = number.toFloat();
  if (param == 'R') mPid->reference(number.toFloat());
  }
  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::changeYawPID(){  // Checks for a change request of P,I,D and Ref from bluetooth
  char charReceived, param;
  String number;

  charReceived=readBluetooth();
  if(charReceived == 'P' || (charReceived == 'I') || (charReceived == 'D') || (charReceived == 'R')){
    param = charReceived;
    
    charReceived=readBluetooth();
    while(charReceived!='*') {
      number += charReceived;
      charReceived=readBluetooth();
    }

  if (param == 'P') mPidAng->mKp = number.toFloat();
  if (param == 'I') mPidAng->mKi = number.toFloat();
  if (param == 'D') mPidAng->mKd = number.toFloat();
  if (param == 'R') mPidAng->reference(number.toFloat());
  }
  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::stepControl(){

  float currentTime = millis();
  mIncT = double(currentTime - mLastTime);

  readSonar(0); // 0 es izquierda y 1 es derecha
  readSonar(1);

  float distanciaMedia = ( mDistSonar[0]+mDistSonar[1] ) / 2.0;
  float outPID    = mPid->update( distanciaMedia , mIncT);

  if (mRobotMode == 3) {  //Si orientacion
    mYaw = atan( ( mDistSonar[0] - mDistSonar[1] )/mDistSensores ) ; // -> grados
    float outAngPID = mPidAng->update( mYaw , mIncT);
    move(outPID,outAngPID);
  } else move(outPID,0);  //si no orientacion

  mLastTime = millis();

  return;
}

//----------------------------------------------------------------------------------
void Golpecitos::stepControlParallel(){

  float currentTime = millis();
  mIncT = double(currentTime - mLastTime);

  readSonar(0); // 0 es izquierda y 1 es derecha
  readSonar(1);
  float distanciaMedia = ( mDistSonar[0]+mDistSonar[1] ) / 2.0;
  float outPID    = mPid->update( distanciaMedia , mIncT);

  mYaw = atan( ( mDistSonar[0] - mDistSonar[1] )/mDistSensores ) ; // -> grados
  float outAngPID = mPidAng->update( mYaw , mIncT);
  
  // if(distanciaMedia != referencia){
  //   move(mVelCrucero,outPID);
  // }else{
  //   move(mVelCrucero,outAngPID);
  // }
  
  mLastTime = millis();

  return;
}

//----------------------------------------------------------------------------------
// log format: mIncT [ms] , distIzq [cm] , distDcha [cm] , ref [cm] , modo [int] , velPWMizq [int] , velPWMdcha [int]
void Golpecitos::writeTelemetry(){

  //String log = String(mIncT)+ " " +String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPid->mKp) + " " + String(mPid->mKi) + " " + String(mPid->mKd)
  //            + " " + String(mPid->reference()) + " " + String(mRobotMode) + " " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";
  String log = String(mIncT)+ " " +String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPidAng->mKp) + " " + String(mPidAng->mKi) + " " + String(mPidAng->mKd)
              + " " + String(mPidAng->reference()) + " " + String(mYaw * (180/M_PI)) + " " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";
  

  Serial1.println(log);

  return;
}
