#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdio.h>
#include <SDL/SDL.h>
#include <math.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <trobot/Odometry.h>

#define min_speed 300
#define max_speed 1000
#define encoder_cpr 3600
#define MISSING_VALUE -1024

using namespace std;

RoboteqDevice device;
string port;

double wheelRadius;


void teleopRwheel(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("I heard rwheel: %f", msg->data);

  device.SetCommand(_GO, 2, msg->data * 1000 / 2);
}

void teleopLwheel(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("I heard lwheel: %f", msg->data);

  device.SetCommand(_GO, 1, msg->data * 1000 / 2);
}

void connect()
{
  int status = RQ_ERR_NOT_CONNECTED;
  while(status != RQ_SUCCESS)
  {
    status = device.Connect(port);

    if(status != RQ_SUCCESS)
    {
      ROS_INFO("Error connecting to device: %i", status);
    }  
  }
}

void setUpEncoders()
{
  int status = device.SetConfig(_MMOD, 1, 0);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_MMOD, 2, 0);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_EMOD, 1, 18);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_EMOD, 2, 34);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_EPPR, 1, encoder_cpr/4);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_EPPR, 2, encoder_cpr/4);
  ROS_DEBUG("Status: %i", status);

  int maxRPM = 130;
  status = device.SetConfig(_MXRPM, 1, maxRPM);
  ROS_DEBUG("Status: %i", status);
  status = device.SetConfig(_MXRPM, 2, maxRPM);
  ROS_DEBUG("Status: %i", status);

  status = device.SetConfig(_RWD, 5000);
  ROS_DEBUG("Status: %i", status);
}

void setupParameters()
{
  ros::NodeHandle n("~");

  n.param("wheel_radius", wheelRadius, 1.0);
  n.param<std::string>("port", port, "port");
}

double convertToLinearVelocity(int rpm)
{
  return rpm *  M_PI * wheelRadius / 30;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "RoboteQNode");

  setupParameters();
  connect();
  setUpEncoders();

  ros::NodeHandle nodeHandler;
  ros::Subscriber sub = nodeHandler.subscribe("rwheel_vtarget", 1, teleopRwheel);
  ros::Subscriber sub2 = nodeHandler.subscribe("lwheel_vtarget", 1, teleopLwheel);
    
  ros::Publisher pub = nodeHandler.advertise<trobot::Odometry>("RoboteQNode/speed", 1);
  ros::Rate loop_rate(10);

  int leftWheelRPM;
  int rightWheelRPM;
  double leftWheelSpeed;
  double rightWheelSpeed;

  while(ros::ok())
  {
    device.GetValue(_ABSPEED, 1, leftWheelRPM);
    device.GetValue(_ABSPEED, 2, rightWheelRPM);

    leftWheelSpeed = convertToLinearVelocity(leftWheelRPM);
    rightWheelSpeed = convertToLinearVelocity(rightWheelRPM);

    ROS_DEBUG("Get RPM:\tleft:\t%i\tright:\t%i", leftWheelRPM, rightWheelRPM);
    ROS_DEBUG("Coverted to:\tleft:\t%f\tright:\t%f", leftWheelSpeed, rightWheelSpeed);

    trobot::Odometry odometry;
    odometry.leftWheelSpeed = leftWheelSpeed;
    odometry.rightWheelSpeed = rightWheelSpeed;

    pub.publish(odometry);

    ros::spinOnce();
    loop_rate.sleep();
  }

  device.Disconnect();
  return 0;
}