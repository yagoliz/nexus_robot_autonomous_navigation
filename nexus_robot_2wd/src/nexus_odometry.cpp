// /** author: Yago Lizarribar
//  ** credits : ros turtlebot node : https://github.com/Arkapravo/turtlebot
//               arduino ros bridge : http://wiki.ros.org/ros_arduino_bridge
// **/
//
#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include "nexus_odometry.h"

NexusOdometry::NexusOdometry(ros::NodeHandle nh, ros::NodeHandle nhp){

  x = 0; y = 0; theta = 0;
  Vu = 0; dtheta = 0; dVx = 0; dVy = 0;

  nhp.param<double>("encoder_pulses"  , encoder_pulses  , 3072.0);
  nhp.param<double>("wheel_diameter"  , wheel_diameter  ,  146.0);
  nhp.param<double>("wheel_separation", wheel_separation,  279.0);

  nhp.param<std::string>("odom_topic", odom_topic,     "/odom");
  nhp.param<std::string>("rps_ropic" , rps_topic , "/rps_odom");
  nhp.param<std::string>("odom_frame", odom_frame,           "odom");
  nhp.param<std::string>("base_frame", base_frame, "base_footprint");

  nhp.param<double>("publish_rate", rate, 100);
  nhp.param<double>("linear_scale_positive" , linear_scale_positive , 1);
  nhp.param<double>("linear_scale_negative" , linear_scale_negative , 1);
  nhp.param<double>("angular_scale_positive", angular_scale_positive, 1);
  nhp.param<double>("angular_scale_negative", angular_scale_negative, 1);

  rps_sub  = nh.subscribe(rps_topic, 1000, &NexusOdometry::obtainOdometry, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
}

void NexusOdometry::obtainOdometry(const geometry_msgs::Vector3Stamped& msg){

  rpsLeft      = msg.vector.x;
  rpsRight     = msg.vector.y;
  dt           = msg.vector.z;
  current_time = msg.header.stamp;

  Vu  = (rpsLeft+rpsRight) * wheel_diameter * PI / (1000*2);
  dtheta = (rpsRight-rpsLeft) * dt * wheel_diameter * PI / wheel_separation;

  if (dtheta > 0) dtheta *= angular_scale_positive;
  if (dtheta < 0) dtheta *= angular_scale_negative;
  if (Vu > 0) Vu *= linear_scale_positive;
  if (Vu < 0) Vu *= linear_scale_negative;

  dVx = cos(dtheta/2) * Vu;
  dVy = sin(dtheta/2) * Vu;

  x += (cos(theta) * dVx - sin(theta) * dVy)*dt;
  y += (sin(theta) * dVx + cos(theta) * dVy)*dt;
  theta += dtheta;

  if(theta >=  2*PI) theta -= 2*PI;
  if(theta <= -2*PI) theta += 2*PI;

  q.setRPY(0.0, 0.0, theta);

  // Odometry publisher
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.child_frame_id = base_frame;
  odom.twist.twist.linear.x = Vu;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = dtheta/dt;

  odom_pub.publish(odom);

  // Transform broadcaster
  transformStamped.header.frame_id = odom_frame;
  transformStamped.child_frame_id  = base_frame;
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  transformStamped.header.stamp = current_time;

  broadcaster.sendTransform(transformStamped);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  NexusOdometry nexusOdometry(nh, nhp);
  ros::spin();

  return 0;
}
