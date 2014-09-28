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
#include <trobot/MotorAmps.h>
#include <trobot/Encoder.h>

#define min_speed 300
#define max_speed 1000
#define encoder_cpr 3600
#define MISSING_VALUE -1024

using namespace std;

int minBatteryVoltage = 68;
int maxBatteryVoltage = 84;

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
  n.param<std::string>("port", port, "/dev/ttyACM2");
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
    
  ros::Publisher speedPub = nodeHandler.advertise<trobot::Odometry>("RoboteQNode/speed", 1);
  ros::Publisher batteryPub = nodeHandler.advertise<std_msgs::Float32>("RoboteQNode/battery", 1);
  ros::Publisher ampsPub = nodeHandler.advertise<trobot::MotorAmps>("RoboteQNode/amps", 1);
  ros::Publisher encoderPub = nodeHandler.advertise<trobot::Encoder>("RoboteQNode/encoder", 1);
  ros::Rate loop_rate(10);

  int leftWheelRPM;
  int rightWheelRPM;
  trobot::Odometry odometry;

  int batteryVoltage;
  std_msgs::Float32 batteryCharge;

  int motorsAmps;
  trobot::MotorAmps motorsAmpsMsgs;

  int wheelCount;
  trobot::Encoder encoderCount;

  while(ros::ok())
  {
    device.GetValue(_ABSPEED, 1, leftWheelRPM);
    device.GetValue(_ABSPEED, 2, rightWheelRPM);

    odometry.leftWheelSpeed = convertToLinearVelocity(leftWheelRPM);
    odometry.rightWheelSpeed = convertToLinearVelocity(rightWheelRPM);

    ROS_DEBUG("Get RPM:\tleft:\t%i\tright:\t%i", leftWheelRPM, rightWheelRPM);
    ROS_DEBUG("Coverted to:\tleft:\t%f\tright:\t%f", odometry.leftWheelSpeed, odometry.rightWheelSpeed);

    speedPub.publish(odometry);


    device.GetValue(_VOLTS, 2, batteryVoltage);
    batteryCharge.data   = (batteryVoltage - minBatteryVoltage) * 100 / (maxBatteryVoltage - minBatteryVoltage);

    ROS_DEBUG("Battery voltage:\t%i", batteryVoltage);
    ROS_DEBUG("Battery charge:\t%f %", batteryCharge.data);

    batteryPub.publish(batteryCharge);


    device.GetValue(_MOTAMPS, 1, motorsAmps);
    motorsAmpsMsgs.leftMotorAmps = motorsAmps / 10;
    device.GetValue(_MOTAMPS, 2, motorsAmps);
    motorsAmpsMsgs.rightMotorAmps = motorsAmps / 10;

    ROS_DEBUG("MotorsAmps1:\t%f", motorsAmpsMsgs.leftMotorAmps);
    ROS_DEBUG("MotorsAmps2:\t%f", motorsAmpsMsgs.rightMotorAmps);

    ampsPub.publish(motorsAmpsMsgs);


    device.GetValue(_ABCNTR, 1, wheelCount);
    encoderCount.leftWheelCount = wheelCount;
    device.GetValue(_ABCNTR, 2, wheelCount);
    encoderCount.rightWheelCount = wheelCount;

    ROS_DEBUG("Left Wheel count:\t%f", encoderCount.leftWheelCount);
    ROS_DEBUG("Right Wheel count:\t%f", encoderCount.rightWheelCount);

    encoderPub.publish(encoderCount);

    ros::spinOnce();
    loop_rate.sleep();
  }

  device.Disconnect();
  return 0;
}