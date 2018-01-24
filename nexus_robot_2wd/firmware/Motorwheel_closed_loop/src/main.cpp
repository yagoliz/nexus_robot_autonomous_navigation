#include <Arduino.h>
#include <PID_v1.h>
#include <Timer.h>
#include <Event.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <ArduinoHardware.h>
#include "nexus_robot_specs.h"


// Robot variables
double rpsLeft = 0, pwmLeft = 0, Enc_rpsLeft = 0;
double encoderL0 = 0;
double encoderL1 = 0;

double rpsRight = 0, pwmRight = 0, Enc_rpsRight = 0;
double encoderR0 = 0;
double encoderR1 = 0;

unsigned long past_time = 0;

// ROS variables
ros::NodeHandle nh;

// Callback function
void get_speed(const geometry_msgs::Vector3& msg)
{
  rpsLeft  = abs(msg.x);
  rpsRight = abs(msg.y);

  if (msg.x>0){digitalWrite(M1, HIGH);}
  else {digitalWrite(M1, LOW);}

  if (msg.y>0){digitalWrite(M2, LOW);}
  else {digitalWrite(M2, HIGH);}

}

ros::Subscriber<geometry_msgs::Vector3> sub("rps", get_speed);

geometry_msgs::Vector3Stamped rps_odometry;
// geometry_msgs::Vector3 debug_msg;
ros::Publisher pub("rps_odom", &rps_odometry);
// ros::Publisher debug("debug_node", &debug_msg);


Timer vel_timer;
Timer ros_publisher;

// PID objects
double Kpl=70.0, Kil=60.0, Kdl=0.4;
double Kpr=60.0, Kir=60.0, Kdr=0.4;
PID myPIDL(&Enc_rpsLeft, &pwmLeft, &rpsLeft, Kpl, Kil, Kdl, DIRECT);
PID myPIDR(&Enc_rpsRight, &pwmRight, &rpsRight, Kpr, Kir, Kdr, DIRECT);

// Interrupts and Timers
void publish_rps();
void obtain_rps_encoder();
void doEncoderlA();
void doEncoderlB();
void doEncoderrA();
void doEncoderrB();

// Main fucntions
void setup()
{
  TCCR4B &= 0xF8;
  TCCR4B |= 0x01;

  // Left Wheel setup
  pinMode(M1,OUTPUT); //M1 direction control
  pinMode(E1,OUTPUT); analogWrite(E1, 0);
  pinMode(ELA, INPUT_PULLUP);
  pinMode(ELB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ELA), doEncoderlA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELB), doEncoderlB, CHANGE);


  // Right Wheel setup
  pinMode(M2,OUTPUT); //M2 direction control
  pinMode(E2,OUTPUT); analogWrite(E2, 0);
  pinMode(ERA, INPUT_PULLUP);
  pinMode(ERB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ERA), doEncoderrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ERB), doEncoderrB, CHANGE);


  vel_timer.every(50, obtain_rps_encoder);
  ros_publisher.every(50, publish_rps);

  myPIDL.SetSampleTime(10);
  myPIDL.SetMode(AUTOMATIC);

  myPIDR.SetSampleTime(10);
  myPIDR.SetMode(AUTOMATIC);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  // nh.advertise(debug);
}

void loop() {
  myPIDL.Compute();
  myPIDR.Compute();

  analogWrite(E1, abs(pwmLeft));
  analogWrite(E2, abs(pwmRight));

  vel_timer.update();
  ros_publisher.update();
  nh.spinOnce();
}

void obtain_rps_encoder()
{
  unsigned long current_time = millis();
  double dt = (current_time - past_time)/1000.0;
  past_time = current_time;

  Enc_rpsLeft = (encoderL1-encoderL0) / PPR / dt;
  rps_odometry.vector.x = Enc_rpsLeft;
  Enc_rpsLeft = abs(Enc_rpsLeft);
  encoderL0 = encoderL1;

  Enc_rpsRight = (encoderR1-encoderR0) / PPR / dt;
  rps_odometry.vector.y = Enc_rpsRight;
  Enc_rpsRight = abs(Enc_rpsRight);
  encoderR0 = encoderR1;

  // debug_msg.x = pwmLeft;
  // debug_msg.y = pwmRight;
  // debug_msg.z = dt;
  rps_odometry.vector.z = dt;
}

void publish_rps()
{
  rps_odometry.header.stamp = nh.now();
  pub.publish(&rps_odometry);
  // debug.publish(&debug_msg);
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
