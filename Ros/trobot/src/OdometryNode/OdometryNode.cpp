#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <trobot/Odometry.h>
#include <trobot/Encoder.h>
#include <math.h>

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double sx = 0.0;
double sy = 0.0;
double sth = 0.0;

double wheelDistance;
double axialDistance;
double alpha;
double wheelRadius;

void getOdometry(const trobot::Odometry::ConstPtr& msg) {
  ROS_INFO("I heard rWheel: %f", msg->rightWheelSpeed);
  ROS_INFO("I heard lWheel: %f", msg->leftWheelSpeed);

  vx = (msg->rightWheelSpeed + msg->leftWheelSpeed) / 2.0;
  vth = (4 * alpha * (msg->rightWheelSpeed - vx)) / (2 * wheelDistance + axialDistance);
}

void getEncoderCount(const trobot::Encoder::ConstPtr& msg) {
  ROS_INFO("I heard rWheelCount: %f", msg->rightWheelCount);
  ROS_INFO("I heard lWheelCount: %f", msg->leftWheelCount);

  if(msg->rightWheelCount > 0 && msg->rightWheelCount > 0) {
    double R = -msg->rightWheelCount * axialDistance / (msg->leftWheelCount - msg->rightWheelCount);
    double a = 4.8 * msg->leftWheelCount * wheelRadius / R;
    double beta = 90 - a / 2;
  
    sx = 2 * R * cos(beta) * sin(beta);
    sy = 2 * R * cos(beta) * cos(beta);
    sth = a * M_PI / 180;
  } else {
    sx = 0;
    sy = 0;
    sth = 0; 
  }
}

void setupParameters() {
  ros::NodeHandle n("~");

  n.param("wheel_distance", wheelDistance, 1.0);
  n.param("axial_distance", axialDistance, 1.0);
  n.param("alpha", alpha, 0.4);
  n.param("wheel_radius", wheelRadius, 0.063);
}

nav_msgs::Odometry createOdomMsg(double x, double y, double th, ros::Time current_time) {
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.pose.covariance[0] = 0.00001;
  odom.pose.covariance[7] = 0.00001;
  odom.pose.covariance[14] = 1000000000000.0;
  odom.pose.covariance[21] = 1000000000000.0;
  odom.pose.covariance[28] = 1000000000000.0;
  odom.pose.covariance[35] = 0.001;
  
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom.twist.covariance = odom.pose.covariance;
	
  return odom;
} 

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  setupParameters();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("RoboteQNode/speed", 1, getOdometry);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("OdometryNode/odom", 50);
  //ros::Subscriber sub2 = n.subscribe("RoboteQNode/encoder", 1, getEncoderCount);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);
  while(n.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();
	
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
	
    odom_pub.publish(createOdomMsg(x, y, th, current_time));
    //odom_pub.publish(createOdomMsg(sx, sy, sth, current_time));

    last_time = current_time;
    r.sleep();
  }
}