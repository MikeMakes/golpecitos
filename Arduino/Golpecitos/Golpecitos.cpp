
#include "Golpecitos.h"

// ----------- Constructor de la clase -----------
Golpecitos::Golpecitos() {
	Serial.println("Se ha llamado al constructor de la clase");
}

// ----------- Destructor de la clase -----------
Golpecitos::~Golpecitos(){}

//  ----------- Metodos(funciones) publicos -----------
void Golpecitos::saluda(){
	Serial.println("hola jeje");
	return ;
}


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
}

float* Golpecitos::cinematica(float _lin, float _ang){
	mSpeed[0]=(_lin-eje*_ang)/r;
	mSpeed[1]=(_lin+eje*_ang)/r;
	return mSpeed;
}

void Golpecitos::move(float _lin, float _ang){ //input _vel[0]=velocidad_rueda_izq _vel[1]=velocidad_rueda_dch
	cinematica(_lin,_ang);
	write_pwm(mL_en, mSpeed[0], mL_a, mL_b);
	write_pwm(mR_en, mSpeed[1], mR_a,mR_b);
}


//  ----------- Metodos(funciones) privados -----------
