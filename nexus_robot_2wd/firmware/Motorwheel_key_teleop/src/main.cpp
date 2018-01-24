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

byte directionRight = LOW; // CW
float rpsRight = 0.0;

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

std_msgs::Float64 rps_value;
ros::Publisher pub("chatter", &rps_value);

Timer vel_timer;
Timer ros_publisher;

int rps2ms(float rps);
void publish_rps();
void set_PWM();

void setup()
{
  pinMode(M1,OUTPUT); //M1 direction control
  pinMode(E1,OUTPUT);
  pinMode(M2,OUTPUT); //M2 direction control
  pinMode(E2,OUTPUT);

  analogWrite(E1, 0);
  analogWrite(E2, 0);

  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  vel_timer.every(20, set_PWM);
  ros_publisher.every(20, publish_rps);

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
  int pwmRight = rps2ms(abs(rpsRight));
  int pwmLeft = rps2ms(abs(rpsLeft));
  rps_value.data = rpsLeft;

  digitalWrite(M1, directionLeft);
  analogWrite(E1, pwmLeft);

  digitalWrite(M2, directionRight);
  analogWrite(E2, pwmRight);
}

// Map RPM to ms
int rps2ms(float rps)
{
  int pwm = 0;
  float rps_corrected = min(MAXRPS, rps);
  pwm = rps_corrected*MAXPWM/MAXRPS;
  return pwm;
}

void publish_rps()
{
  //rpm_value.data = (int) rpmRight;
  pub.publish(&rps_value);
}
