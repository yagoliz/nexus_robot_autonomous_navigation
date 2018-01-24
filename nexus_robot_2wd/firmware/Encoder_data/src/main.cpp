#include <Arduino.h>
#include <Timer.h>
#include "nexus_robot_specs.h"

int encoderL0 = 0;
int encoderL1 = 0;

int encoderR0 = 0;
int encoderR1 = 0;

Timer t;
void printstuff();
void doEncoderlA();
void doEncoderlB();
void doEncoderrA();
void doEncoderrB();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(57600);
    pinMode(50,OUTPUT); digitalWrite(50,HIGH);//M1 direction control
    pinMode(4,OUTPUT); analogWrite(4, 100);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), doEncoderlA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), doEncoderlB, CHANGE);
    // Right Wheel setup
    pinMode(52,OUTPUT); digitalWrite(52,LOW);//M2 direction control
    //pinMode(5,OUTPUT); analogWrite(5, 100);
    pinMode(ERA, INPUT_PULLUP);
    pinMode(ERB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ERA), doEncoderrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ERB), doEncoderrB, CHANGE);

    t.every(100, printstuff);

}

void loop() {
    t.update();
}

void printstuff()
{
  Serial.print("blah "); Serial.print(encoderL1-encoderL0); Serial.print(", "); Serial.println(encoderR1-encoderR0);
  encoderL0 = encoderL1;
  encoderR0 = encoderR1;
}

void doEncoderlA()
{
  if (digitalRead(2)==HIGH){
    if (digitalRead(3)==LOW) encoderL1 = encoderL1 + 1; //CCW -
    else encoderL1 = encoderL1 - 1; //CW
  }
  else{
    if (digitalRead(3)==LOW) encoderL1 = encoderL1 - 1; //CW
    else encoderL1 = encoderL1 + 1; //CCW -
  }
}

void doEncoderlB()
{
  if (digitalRead(3)==HIGH){
    if (digitalRead(2)==LOW) encoderL1 = encoderL1 - 1; //CW
    else encoderL1 = encoderL1 + 1; //CCW -
  }
  else{
    if (digitalRead(2)==LOW) encoderL1 = encoderL1 + 1; //CCW -
    else encoderL1 = encoderL1 - 1; //CW
  }
}

void doEncoderrA()
{
  if (digitalRead(18)==HIGH){
    if (digitalRead(19)==LOW) encoderR1 = encoderR1 - 1; //CCW
    else encoderR1 = encoderR1 + 1; //CW
  }
  else{
    if (digitalRead(19)==LOW) encoderR1 = encoderR1 + 1; //CW
    else encoderR1 = encoderL1 - 1; //CCW
  }
}

void doEncoderrB()
{
  if (digitalRead(19)==HIGH){
    if (digitalRead(18)==LOW) encoderR1 = encoderR1 + 1; //CCW
    else encoderR1 = encoderR1 - 1; //CCW
  }
  else{
    if (digitalRead(18)==LOW) encoderR1 = encoderR1 - 1; //CCW
    else encoderR1 = encoderR1 + 1; //CW
  }}
