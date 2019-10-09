
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos(int _pinEcho,int _pinTrig) {
	Serial.println("Se ha llamado al constructor de la clase");

  mPinEcho = _pinEcho;
  mPinTrig = _pinTrig;
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
  pinMode(mPinTrig, OUTPUT);
  // Ponemos el pin Echo en modo entrada
  pinMode(mPinEcho, INPUT);

  /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT);  pinMode(mR_en, OUTPUT);

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
    }    else{
        digitalWrite(_enable, LOW);
    }

    return;
}


//----------------------------------------------------------------------------------
void Golpecitos::cinematica(float _lin, float _ang){
	mSpeed[0]=(_lin-eje*_ang)/r;
	mSpeed[1]=(_lin+eje*_ang)/r;

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
void Golpecitos::readSonar(int _sonarNum){
  iniciarTrigger();
  
  // La función pulseIn obtiene el tiempo que tarda en cambiar entre estados, en este caso a HIGH
  unsigned long tiempo = pulseIn(mPinEcho, HIGH);
    // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001
  mDistSonar = tiempo * 0.000001 * mVelSon / 2.0;
  Serial.print(mDistSonar);
  Serial.print("cm");
  Serial.println();
  delay(1000);

  return;
}


// Método que inicia la secuencia del Trigger para comenzar a medir
void Golpecitos::iniciarTrigger(){
  // Ponemos el Triiger en estado bajo y esperamos 2 ms
  digitalWrite(mPinTrig, LOW);
  delayMicroseconds(2);
  
  // Ponemos el pin Trigger a estado alto y esperamos 10 ms
  digitalWrite(mPinTrig, HIGH);
  delayMicroseconds(10);
  
  // Comenzamos poniendo el pin Trigger en estado bajo
  digitalWrite(mPinTrig, LOW);
}
