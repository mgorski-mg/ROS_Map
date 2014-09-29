#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Float32.h"
#include <math.h>

float32[] ranges;

void getScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ranges = msg->ranges;
}

void setupParameters() {
  ros::NodeHandle n("~");

  n.param("wheel_distance", wheelDistance, 1.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  setupParameters();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1, getScan);
  ros::Publisher l_pub = n.advertise<std_msgs::Float32>("lwheel_vtarget", 50);
  ros::Publisher r_pub = n.advertise<std_msgs::Float32>("rwheel_vtarget", 50);

  ros::Rate r(10);
  while(n.ok()) {
    ros::spinOnce();

	int size = sizeof(range);
	int firstEnd = round((size - 150) / 2) - 1;
	int secondEnd = firstEnd + 150;
	int thirdEnd = size - 1;
	
	bool isFirstEmpty = true;
	bool isSecondEmpty = true;
	bool isThirdEmpty = true;
	
	for(int i; i < size; i++) {
		float distance = range[i];
		if(distance < 0.8) {
			if(i <= firstEnd) {
				isFirstEmpty = false;
				i = firstEnd + 1;
			} else if(i <= secondEnd) {
				isSecondEmpty = false;
				i = secondEnd + 1;
			} else if(i <= thirdEnd) {
				isThirdEmpty = false;
				break;
			}
		}
	}
	
	std_msgs::Float32 lwheel;
	std_msgs::Float32 rwheel;
	
	if(isSecondEmpty) {
		lwheel.data = 0.8;
		rwheel.data = 0.8;
	} else if(isFirstEmpty) {
		lwheel.data = 0.8;
		rwheel.data = -0.8;
	} else {
		lwheel.data = -0.8;
		rwheel.data = 0.8;
	}
	
    l_pub2.publish(lwheel);
	r_pub.publish(rwheel);

    r.sleep();
  }
}