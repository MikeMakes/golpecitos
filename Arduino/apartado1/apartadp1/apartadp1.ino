/*
ENABLE(PWM) |      A       |     B        |   STATUS           |
--------------------------------------------------------- |
     LOW    |    ----      |    ----      |MOTOR PARADO        |
     HIGH   |    HIGH      |     LOW      |MOTOR GIRA DELANTE  |
     HIGH   |    LOW       |     HIGH     |MOTOR GIRA AL REVES |
     HIGH   |    HIGH      |     HIGH     |MOTOR PARADO        |
     HIGH   |    LOW       |     LOW      |MOTOR PARADO        |
 */

/* Define the Motors pin numbers */
#define mL_en 2
#define mR_en 11
// Motors directions
#define mL_a 3
#define mL_b 4
#define mR_a 13
#define mR_b 12

// Define robot distances
#define r 3.15 // Radio de la rueda
#define k 6.425 //Distancia del centro a la rueda
#define v_max 800//Velocidad maxima a la que puede ir para una velocidad puramente lineal de 8 bits (255)
#define v_crucero 300 //Velocidad de crucero
// Global variables
float w[2]; // Wheels velocities {left, right}

void write_pwm(int enable,int pwm, int dir1, int dir2){ //function to write pwm 2 motors & manage directions

    if(pwm > 0){
      if (pwm>255) pwm=255;
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        analogWrite(enable, pwm);
    }
    else if(pwm < 0){
      if (pwm<-255) pwm=-255;
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(enable, abs(pwm));
    }
    else{
        digitalWrite(enable, LOW);
    }
}

void move(){  // escribe la velocidad w, generalmente calculada
  write_pwm(mR_en,w[1], mR_a,mR_b);
  write_pwm(mL_en, w[0], mL_a, mL_b);
}

void vel(float lin, float ang){ //modelo cinematico
 float acoplamiento = ang / lin;
 w[0]=(lin-k*acoplamiento)/r;
 w[1]=(lin+k*acoplamiento)/r;  
}

void setup() {
    /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT); pinMode(mR_en, OUTPUT);

}
// 1. Montar y programar el robot para realizar los movimientos básicos de una configuración
// diferencial:
// a. Avanzar en línea recta
// b. Retroceder
// c. Rotar en sentido horario
// d. Rotar en sentido antihorario
// e. Girar Izquierda (hacia adelante y atrás)
// f. Girar Derecha (hacia adelante y atrás)
void loop() {
//a. Avanzar en línea recta
  vel(v_crucero,0);  // un pasito palante
  move();
  delay(2000);
// b. Retroceder
  vel(-v_crucero,0); // un pasito pa tras
  move();
  delay(2000);
// c. Rotar en sentido horario
  vel(0,v_crucero);
  move();
  delay(2000); 
// d. Rotar en sentido antihorario
  vel(0,-v_crucero);
  move();
  delay(2000); 
// e. Girar Izquierda (hacia adelante y atrás)
  vel(v_max,-v_crucero);
  move();
  delay(2000);
  vel(-v_max,-v_crucero);
  move();
  delay(2000);  
// f. Girar Derecha (hacia adelante y atrás)
  vel(v_max,v_crucero);
  move();
  delay(2000);
  vel(v_max,v_crucero);
  move();
  delay(2000);  
}
