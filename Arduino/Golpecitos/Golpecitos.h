#ifndef Golpecitos_h
#define Golpecitos_h

#include <Arduino.h>
#include "PID.h"

/*
ENABLE(PWM) |      A       |     B        |   STATUS           |
--------------------------------------------------------- |
     LOW    |    ----      |    ----      |MOTOR PARADO        |
     HIGH   |    HIGH      |     LOW      |MOTOR GIRA DELANTE  |
     HIGH   |    LOW       |     HIGH     |MOTOR GIRA AL REVES |
     HIGH   |    HIGH      |     HIGH     |MOTOR PARADO        |
     HIGH   |    LOW       |     LOW      |MOTOR PARADO        |
 */

class Golpecitos{

  public:
    Golpecitos(int _pinEchoIzq,int _pinTrigIzq,int _pinEchoDcha,int _pinTrigDcha);
    ~Golpecitos();

    void inicialize();
    void saluda();
    void cinematica(float lin, float ang) ;
    void move(float _lin, float _ang);
    void write_pwm(int _enable,int _pwm, int _dir1, int _dir2);


    float readSonar(int _sonarNum);
    char readBluetooth();
    void step();
    void stepControl();

  public:
    float mSpeed[2]; // Velocidad de las ruedas: 0-izq y 1-dch
    float mDistSonar;



    PID *mPid;
    float mLastTime = 0.0; // used in controller

  private:
    float mWheelRadius = 3.15; // Radio de las ruedas
    float mDistWheels  = 6.425; // Longitud del eje / 2

    int mPinTrigIzq  = 7;
    int mPinEchoIzq  = 6;
    int mPinTrigDcha = 7;
    int mPinEchoDcha = 6;

    // Define the Motors pin numbers (velocity - pwm) 
    int mL_en = 2;
    int mR_en = 11;
    // Motors directions
    int mL_a = 3;
    int mL_b = 4;
    int mR_a = 13;
    int mR_b = 12;

    float mVelMax     = 800.0; // equivale a 255 pwm
    float mVelCrucero = 300.0;

    // Variable to save bluetooth data
    char mBluetoothData;
    
    // used in sonars
    const float mVelSon = 34000.0;

  private:
    // Método que inicia la secuencia del Trigger para comenzar a medir
    void iniciarTrigger();

};


#endif
