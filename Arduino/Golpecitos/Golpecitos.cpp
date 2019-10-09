
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha) {
	Serial.println("Se ha llamado al constructor de la clase");

  mPinEchoIzq = _pinEchoIzq;
  mPinTrigIzq = _pinTrigIzq;

  mPinEchoIzq = _pinEchoIzq;
  mPinTrigIzq = _pinTrigIzq;

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
  pinMode(mPinTrigIzq, OUTPUT);
  // Ponemos el pin Echo en modo entrada
  pinMode(mPinEchoIzq, INPUT);

  /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT);  pinMode(mR_en, OUTPUT);

  // Configure bluetooth seral
  Serial1.begin(38400);


  // Configure controller pointer
  mPid = new PID(1.0, 0.0 , 0.0 ,0.0,10.0);
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
	write_pwm(mR_en, mSpeed[1], mR_a,mR_b);

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
  digitalWrite(_pintTrig, LOW);
  delayMicroseconds(2);
  
  // Ponemos el pin Trigger a estado alto y esperamos 10 ms
  digitalWrite(_pintTrig, HIGH);
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


void Golpecitos::stepControl(){

  // Feed PIDs
  float currentTime = millis();
  double incT = double(currentTime - mLastTime);

  float distanciaSonar1 = readSonar(1);
  float outPID = mPid->update( distanciaSonar1 , incT); // entrada -> medida ; salida -> (?)

  // Aqui se deberia actuar con la salida del control
  //
  //
  // 
  mLastTime = millis();
}
