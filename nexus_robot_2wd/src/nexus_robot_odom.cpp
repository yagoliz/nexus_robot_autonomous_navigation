// /** author: Sung Jik Cha
//  ** credits : ros turtlebot node : https://github.com/Arkapravo/turtlebot
//               arduino ros bridge : http://wiki.ros.org/ros_arduino_bridge
// **/
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include "nexus_robot_params.h"

// General variables
double rps_Left = 0.0;
double rps_Right = 0.0;
double dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
ros::Time rps_time(0.0);
ros::Time last_time(0.0);

void handle_rps( const geometry_msgs::Vector3Stamped& rps) {
  rps_Left = rps.vector.x;
  rps_Right = rps.vector.y;
  dt = rps.vector.z;
  rps_time = rps.header.stamp;
}

// void handle_gyro( const geometry_msgs::Vector3& gyro) {
//   gyro_x = gyro.x;
//   gyro_y = gyro.y;
//   gyro_z = gyro.z;
// }

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("rps_odom", 50, handle_rps);
//  ros::Subscriber gyro_sub = n.subscribe("gyro", 50, handle_gyro);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  // Parameter definition
  double rate = 10.0;
  double linear_scale_positive = 1.0;
  double linear_scale_negative = 1.0;
  double angular_scale_positive = 1.0;
  double angular_scale_negative = 1.0;
  double angular_scale_accel = 1.0;
  double alpha = 1.0;
  bool publish_tf = true;
  // Odometry variables
  // double Wz_prev = 0.0;
  // double Wz_curr = 0.0;
  // double Vu_prev = 0.0;
  double Vu_ave = 0.0;
  double dVx = 0.0;
  double dVy = 0.0;
  double dWz_odom = 0.0;
  // IMU variables
  bool use_imu = false;
  double dWz_gyro = 0.0;
  double gyro_x = 0;
  double gyro_y = 0;
  double gyro_z = 0;
  // Combined variables
  double dWz = 0.0;
  double Vx = 0.0;
  double Vy = 0.0;
  double W = 0.0;
  // TF frames
  char base_link[] = "/base_footprint";
  char odom[] = "/odom";
  ros::Duration d(1.0);
  // Set parameters
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);
  nh_private_.getParam("angular_scale_accel", angular_scale_accel);
  nh_private_.getParam("alpha", alpha);
  nh_private_.getParam("use_imu", use_imu);

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rps_odom", n, d);
    current_time = ros::Time::now();

    Vu_ave   = (rps_Left+rps_Right) * WD * PI / (1000*2);
    dWz_odom = (rps_Right-rps_Left) * dt * WD * PI / WS ;

    if (use_imu) dWz_gyro = gyro_z;
    dWz = alpha*dWz_odom + (1-alpha)*dWz_gyro;

    if (dWz > 0) dWz *= angular_scale_positive;
    if (dWz < 0) dWz *= angular_scale_negative;
    if (Vu_ave > 0) Vu_ave *= linear_scale_positive;
    if (Vu_ave > 0) Vu_ave *= linear_scale_negative;

    // dVx = cos(dWz) * Vu_ave;
    // dVy = sin(dWz) * Vu_ave;
    dVx = Vu_ave;
    dVy = 0;

    x_pos += (cos(theta) * dVx - sin(theta) * dVy)*dt;
    y_pos += (sin(theta) * dVx + cos(theta) * dVy)*dt;
    theta += dWz;

    if(theta >=  2*PI) theta -= 2*PI;
    if(theta <= -2*PI) theta += 2*PI;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;

      broadcaster.sendTransform(t);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (rps_Left == 0 && rps_Right == 0){
      // odom_msg.pose.covariance[0] = 1e-9;
      // odom_msg.pose.covariance[7] = 1e-3;
      // odom_msg.pose.covariance[8] = 1e-9;
      // odom_msg.pose.covariance[14] = 1e6;
      // odom_msg.pose.covariance[21] = 1e6;
      // odom_msg.pose.covariance[28] = 1e6;
      // odom_msg.pose.covariance[35] = 1e-9;
      // odom_msg.twist.covariance[0] = 1e-9;
      // odom_msg.twist.covariance[7] = 1e-3;
      // odom_msg.twist.covariance[8] = 1e-9;
      // odom_msg.twist.covariance[14] = 1e6;
      // odom_msg.twist.covariance[21] = 1e6;
      // odom_msg.twist.covariance[28] = 1e6;
      // odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      // odom_msg.pose.covariance[0] = 1e-3;
      // odom_msg.pose.covariance[7] = 1e-3;
      // odom_msg.pose.covariance[8] = 0.0;
      // odom_msg.pose.covariance[14] = 1e6;
      // odom_msg.pose.covariance[21] = 1e6;
      // odom_msg.pose.covariance[28] = 1e6;
      // odom_msg.pose.covariance[35] = 1e3;
      // odom_msg.twist.covariance[0] = 1e-3;
      // odom_msg.twist.covariance[7] = 1e-3;
      // odom_msg.twist.covariance[8] = 0.0;
      // odom_msg.twist.covariance[14] = 1e6;
      // odom_msg.twist.covariance[21] = 1e6;
      // odom_msg.twist.covariance[28] = 1e6;
      // odom_msg.twist.covariance[35] = 1e3;
    }
    Vx = Vu_ave;
    W = dWz;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = Vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = W;

    odom_pub.publish(odom_msg);
    last_time = current_time;
    r.sleep();
  }
}
