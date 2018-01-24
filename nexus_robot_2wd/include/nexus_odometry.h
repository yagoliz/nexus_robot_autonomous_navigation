#ifndef NEXUS_ODOMETRY
#define NEXUS_ODOMETRY

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.1415926535897932384626

class NexusOdometry{

  public:
    NexusOdometry(ros::NodeHandle nh, ros::NodeHandle nhp);

  private:
    ros::Time current_time;

    double encoder_pulses, wheel_diameter, wheel_separation;

    ros::Publisher odom_pub;
    ros::Subscriber rps_sub;
    std::string rps_topic;
    std::string odom_topic;

    double rpsLeft, rpsRight;

    double dt, x, y, theta, Vu, dtheta, dVx, dVy;
    double rate, linear_scale_positive, linear_scale_negative, angular_scale_positive, angular_scale_negative;

    std::string odom_frame;
    std::string base_frame;

    nav_msgs::Odometry odom;

    tf2_ros::TransformBroadcaster broadcaster;
    tf2::Quaternion q;
    geometry_msgs::TransformStamped transformStamped;

    void obtainOdometry(const geometry_msgs::Vector3Stamped& msg);
};

#endif
