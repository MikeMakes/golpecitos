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

void move_right(int pwm){
  write_pwm(mR_en,pwm, mR_a,mR_b);
}

void move_left(int pwm){
  write_pwm(mL_en, pwm, mL_a, mR_b);
}

void setup() {
    /* Motors pin init */
  pinMode(mL_a, OUTPUT);   pinMode(mL_b, OUTPUT);
  pinMode(mR_a, OUTPUT);   pinMode(mR_b, OUTPUT);
  pinMode(mL_en, OUTPUT); pinMode(mR_en, OUTPUT);

}

void loop() {
  move_left(255);
  move_right(255);

}
