
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha) {
	Serial.println("Se ha llamado al constructor de la clase");

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
  mPid =    new PID(-1.0  , 0.01 , 0.0 , -20.0,20.0 , 50, -50); //P -100 funciona: P-75,I-0.5 funciona:
  mPidAng = new PID(60.0 , 0.1 , 0.0 , -803.0,803.0 , 50, -50);//SATURAR EL PWM ANGULAR PARA QUE NO SE VUELVA LOCO
  
  mPid->reference(50.0);
  mPidAng->reference(0.0);

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
      move(mVelMax, mVelMax);
      break;

    case '5':  // girar derecha
      move(mVelMax , -mVelMax);
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

//----------------------------------------------------------------------------------
void Golpecitos::changeState(){
  char charReceived;
  charReceived = readBluetooth();

  if(charReceived == 'M'){
    charReceived = readBluetooth(); //pasamos a int
    mRobotMode =  charReceived - '0';
  }
    
}

//----------------------------------------------------------------------------------
void Golpecitos::changePID(){  // Checks for a change request of P,I,D from bluetooth
  //Identificar kd,kp,o ki
  char charReceived, parametro;
  String number;

  charReceived=readBluetooth();
  if(charReceived == 'P' || (charReceived == 'I') || (charReceived == 'D') || (charReceived == 'R')){
    parametro = charReceived;
    
    charReceived=readBluetooth();
    while(charReceived!='*') {
      number += charReceived;
      charReceived=readBluetooth();
    }

  if (parametro == 'P') mPid->mKp = number.toFloat();
  if (parametro == 'I') mPid->mKi = number.toFloat();
  if (parametro == 'D') mPid->mKd = number.toFloat();
  if (parametro == 'R') mPid->reference(number.toFloat());
  
  }
}

//----------------------------------------------------------------------------------
void Golpecitos::changeYawPID(){  // Checks for a change request of P,I,D from bluetooth
  //Identificar kd,kp,o ki
  char charReceived, parametro;
  String number;

  charReceived=readBluetooth();
  if(charReceived == 'P' || (charReceived == 'I') || (charReceived == 'D') || (charReceived == 'R')){
    parametro = charReceived;
    
    charReceived=readBluetooth();
    while(charReceived!='*') {
      number += charReceived;
      charReceived=readBluetooth();
    }

  if (parametro == 'P') mPidAng->mKp = number.toFloat();
  if (parametro == 'I') mPidAng->mKi = number.toFloat();
  if (parametro == 'D') mPidAng->mKd = number.toFloat();
  if (parametro == 'R') mPidAng->reference(number.toFloat());
  
  }
}

//----------------------------------------------------------------------------------
void Golpecitos::stepControl(){

  // Feed PIDs
  float currentTime = millis();
  mIncT = double(currentTime - mLastTime);

  readSonar(0); // 0 es izquierda y 1 es derecha
  readSonar(1);

  float distanciaMedia = ( mDistSonar[0]+mDistSonar[1] ) / 2.0;
  float outPID    = mPid->update( distanciaMedia , mIncT); // entrada -> medida ; salida -> (?)

  if (mRobotMode == 3) {  //Si orientacion
    mYaw = atan( ( mDistSonar[0] - mDistSonar[1] )/mDistSensores ) ; // -> grados
    float outAngPID = mPidAng->update( mYaw , mIncT);
    move(outPID,outAngPID);
  } else move(outPID,0.0);  //si no orientacion

  mLastTime = millis();
}

//----------------------------------------------------------------------------------
void Golpecitos::stepControlParallel(){
   // Feed PIDs
  float currentTime = millis();
  mIncT = double(currentTime - mLastTime);

  readSonar(0); // 0 es izquierda y 1 es derecha
  readSonar(1);
  float distanciaMedia = ( mDistSonar[0]+mDistSonar[1] ) / 2.0;
  float outPID    = mPid->update( distanciaMedia , mIncT); // entrada -> medida ; salida -> (?)

  mYaw = atan( ( mDistSonar[0] - mDistSonar[1] )/mDistSensores ) ; // -> grados
  float outAngPID = mPidAng->update( mYaw , mIncT);

  if(mRobotMode == 4){
    if((distanciaMedia > mPid->reference() * 1.1) || (distanciaMedia < mPid->reference() * 0.9)){ // out of ref
      move(mVelMax,outPID);
    }else{
      move(mVelMax,outAngPID);
    }
  }
  else if (mRobotMode == 3){
    move(mVelMax,outAngPID);
  }

  mLastTime = millis();
}

void Golpecitos::writeTelemetry(){
  // Definir string para mandar Aqui
  // log -> mIncT [ms] , distIzq [cm] , distDcha [cm] , ref [cm] , modo [int] , velPWMizq [int] , velPWMdcha [int]
  //String log = String(mIncT)+ " " +String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPid->mKp) + " " + String(mPid->mKi) + " " + String(mPid->mKd)
  //            + " " + String(mPid->reference()) + " " + String(mRobotMode) + " " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";
  //String log = String(mIncT)+ " " +String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPidAng->mKp) + " " + String(mPidAng->mKi) + " " + String(mPidAng->mKd)
   //           + " " + String(mPidAng->reference()) + " " + String(mYaw * (180/M_PI)) + " " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";
/*STRINGS PARA MANDAR
  1-String(mIncT) 
  2-String(mDistSonar[0])
  3-String(mDistSonar[1])
  4-String(mPid->reference())
  5-String(distanciaMedia)
  6-String(mPidAng->reference())
  7-String(mYaw * (180/M_PI))
  8-String(mRobotMode)
  9-String(mSpeed[0])
  10-String(mSpeed[1])
*/
 String log = String(mIncT)+ " " +String(mDistSonar[0]) + " " + String(mDistSonar[1]) + " " + String(mPid->reference()) +" "+ String(( mDistSonar[0]+mDistSonar[1] ) / 2.0)+" " + 
            String(mPidAng->reference()) + " " + String(mYaw * (180/M_PI)) +" "+String(mRobotMode)+" " + String(mSpeed[0]) + " " +  String(mSpeed[1]) + " \n";

  Serial1.println(log);

  return;
}
