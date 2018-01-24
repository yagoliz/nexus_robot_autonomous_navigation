#include <Arduino.h>

int E1= 5;
int M1= 52;

int E2 = 4;
int M2 = 50;

void setup()
{
  pinMode(M1,OUTPUT); //M2 direction control
  pinMode(E1,OUTPUT);
  pinMode(M2,OUTPUT); //M2 direction control
  pinMode(E2,OUTPUT);
  digitalWrite(M1, HIGH);
  //E2 PWM speed control
  analogWrite(E1,100);
  //analogWrite(E2,100);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
  //set the timer1 as the work intrrupt timer
  // to use the timer will defualt at the function of setup;
 }
void loop() { }
