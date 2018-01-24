#include <Arduino.h>
#include <PID_v1.h>
#include <Timer.h>
#include <Event.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <ArduinoHardware.h>
#include "nexus_robot_specs.h"

// Robot pins

byte directionLeft = HIGH; // CCW
float rpsLeft = 0.0;
float Enc_rpsLeft = 0.0;
int encoderL0 = 0;
int encoderL1 = 0;

byte directionRight = LOW; // CW
float rpsRight = 0.0;
float Enc_rpsRight = 0.0;
int encoderR0 = 0;
int encoderR1 = 0;

unsigned long time_now = 0;
unsigned long prev_time = 0;

// ROS variables
ros::NodeHandle nh;

void get_speed(const geometry_msgs::Vector3& msg)
{
  rpsLeft  = msg.x;
  rpsRight = msg.y;

  if (rpsLeft>0){directionLeft = HIGH;}
  else {directionLeft = LOW;}

  if (rpsRight>0){directionRight = LOW;}
  else {directionRight = HIGH;}
}

ros::Subscriber<geometry_msgs::Vector3> sub("rps", get_speed);

geometry_msgs::Vector3 rps_value;
ros::Publisher pub("chatter", &rps_value);

Timer vel_timer;
Timer ros_publisher;

int rps2ms(float rps);
void publish_rps();
void set_PWM();
void doEncoderlA();
void doEncoderlB();
void doEncoderrA();
void doEncoderrB();

void setup()
{
  pinMode(M1,OUTPUT); //M1 direction control
  pinMode(E1,OUTPUT);
  pinMode(ELA, INPUT_PULLUP);
  pinMode(ELB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ELA), doEncoderlA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELB), doEncoderlB, CHANGE);

  pinMode(M2,OUTPUT); //M2 direction control
  pinMode(E2,OUTPUT);
  pinMode(ERA, INPUT_PULLUP);
  pinMode(ERB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ERA), doEncoderrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ERB), doEncoderrB, CHANGE);

  analogWrite(E1, 0);
  analogWrite(E2, 0);

  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  vel_timer.every(50, set_PWM);
  ros_publisher.every(50, publish_rps);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {
  vel_timer.update();
  ros_publisher.update();
  nh.spinOnce();
}

void set_PWM()
{
  int pwmRight = rps2ms(rpsRight);
  int pwmLeft = rps2ms(rpsLeft);

  time_now = millis();
  double dt = (time_now-prev_time)/1000.0;
  prev_time = time_now;
  Enc_rpsLeft = (encoderL1 - encoderL0) / PPR / dt;
  Enc_rpsRight = (encoderR1 - encoderR0) / PPR / dt;
  rps_value.x = Enc_rpsLeft;
  rps_value.y = Enc_rpsRight;
  rps_value.z = dt;

  prev_time = time_now;
  encoderL0 = encoderL1; encoderR0 = encoderR1;

  digitalWrite(M1, directionLeft);
  analogWrite(E1, abs(pwmLeft));

  digitalWrite(M2, directionRight);
  analogWrite(E2, abs(pwmRight));
}

// Map RPM to ms
int rps2ms(float rps)
{
  int pwm = 0;
  float rps_corrected = min(MAXRPS, abs(rps));
  pwm = rps_corrected*MAXPWM/MAXRPS;
  return pwm;
}

void publish_rps()
{
  pub.publish(&rps_value);
}

void doEncoderlA()
{
  if (digitalRead(ELA)==HIGH){
    if (digitalRead(ELB)==LOW) encoderL1 = encoderL1 - 1; //CCW -
    else encoderL1 = encoderL1 + 1; //CW
  }
  else{
    if (digitalRead(ELB)==LOW) encoderL1 = encoderL1 + 1; //CW
    else encoderL1 = encoderL1 - 1; //CCW -
  }
}

void doEncoderlB()
{
  if (digitalRead(ELB)==HIGH){
    if (digitalRead(ELA)==LOW) encoderL1 = encoderL1 + 1; //CW
    else encoderL1 = encoderL1 - 1; //CCW -
  }
  else{
    if (digitalRead(ELA)==LOW) encoderL1 = encoderL1 - 1; //CCW -
    else encoderL1 = encoderL1 + 1; //CW
  }
}

void doEncoderrA()
{
  if (digitalRead(ERA)==HIGH){
    if (digitalRead(ERB)==LOW) encoderR1 = encoderR1 + 1; //CCW
    else encoderR1 = encoderR1 - 1; //CW
  }
  else{
    if (digitalRead(ERB)==LOW) encoderR1 = encoderR1 - 1; //CW
    else encoderR1 = encoderR1 + 1; //CCW
  }
}

void doEncoderrB()
{
  if (digitalRead(ERB)==HIGH){
    if (digitalRead(ERA)==LOW) encoderR1 = encoderR1 - 1; //CCW
    else encoderR1 = encoderR1 + 1; //CCW
  }
  else{
    if (digitalRead(ERA)==LOW) encoderR1 = encoderR1 + 1; //CCW
    else encoderR1 = encoderR1 - 1; //CW
  }
}
