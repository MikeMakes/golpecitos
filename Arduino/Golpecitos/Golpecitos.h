#ifndef Golpecitos_h
#define Golpecitos_h

#include <Arduino.h>
#include "PID.h"

//#include <string.h> //Librerias para las pasar los char a strings y de ahi a floats 
//#include <stdlib.h>

class Golpecitos{
  public:
    Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha);

    void inicialize();
    void step();
    void stepControl();
    void stepControlParallel();

    void writeTelemetry();

    void changeState();
    void changePID();
    void changeYawPID();
    void Stop();

  private:
    void cinematica(float lin, float ang) ;
    void move(float _lin, float _ang);
    void write_pwm(int _enable,int _pwm, int _dir1, int _dir2);

    char readBluetooth();
    float readSonar(int _sonarNum);
    float orienta();

    // Método que inicia la secuencia del Trigger para comenzar a medir
    void iniciarTrigger(int _pinTrig);

  public:
    // modo del robot [0 - parado ; 1 - control frontal ; 2 - control frontal perpendicular ; 3 - control lateral ; 4 - control lateral paralelo]
    int mRobotMode = 0;

  private:
    float mWheelRadius = 3.15; // Radio de las ruedas
    float mDistWheels  = 6.425; // Longitud del eje / 2
    float mDistSensores = 12.0; //Distancia de sensor a sensor en cm

    int mPinTrig[2] = {7,9};  //pines de trigger del ultrasonido por defecto
    int mPinEcho[2] = {6,8};  ////pines de echo del ultrasonido por defecto

    // Se definen los pines de los motores (velocity - pwm) 
    int mL_en = 2;
    int mR_en = 11;
    // Pines de direccion de los motores
    int mL_a = 3;
    int mL_b = 4;
    int mR_a = 13;
    int mR_b = 12;

    float mVelMax     = 400.0; // 800.0 equivale a 255 pwm
    float mVelCrucero = 150.0;

    // Variable usada para guardar el dato del bluetooth
    char mBluetoothData;
    
    // used in sonars
    const float mVelSon = 34000.0;

    float mSpeed[2] = {0.0 , 0.0}; // Velocidad de las ruedas: 0-izq y 1-dch
    float mDistSonar[2];
    float mYaw; //Angulo de orientacion en el plano xy respecto a una supuesta superficie plana frente a los sensores
  
    PID *mPid     = nullptr;
    PID *mPidAng  = nullptr;
    double mIncT=0;
    float mLastTime = 0.0;

};


#endif
