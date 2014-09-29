#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Float32.h"
#include <math.h>

std::vector<float> ranges;

void getScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ranges = msg->ranges;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "autonomic_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1, getScan);
  ros::Publisher l_pub = n.advertise<std_msgs::Float32>("lwheel_vtarget", 50);
  ros::Publisher r_pub = n.advertise<std_msgs::Float32>("rwheel_vtarget", 50);

  ros::Rate r(10);
  while(n.ok()) {
    ros::spinOnce();

    int size = ranges.size();
        
    printf("Start\n");
    printf("Size:\t%i\n", size);
    int firstEnd = round((size - 153) / 2) - 1;
    int secondEnd = firstEnd + 153;
    int thirdEnd = size - 1;
    printf("FirstEnd:\t%i\n", firstEnd);
    printf("SecondEnd:\t%i\n", secondEnd);
    printf("ThirdEnd:\t%i\n", thirdEnd);
	
    bool isFirstEmpty = true;
    bool isSecondEmpty = true;
    bool isThirdEmpty = true;
	
    for(int i = 85; i < size - 85; i++) {
    	float distance = ranges.at(i);
	if(distance < 0.5) {
        printf("Distance:\t%f\n", distance);
		if(i <= firstEnd) {
			printf("First:\t%i\n", i);
			isFirstEmpty = false;
			i = firstEnd + 1;
		} else if(i <= secondEnd) {
			printf("Second:\t%i\n", i);
			isSecondEmpty = false;
			i = secondEnd + 1;
		} else if(i <= thirdEnd) {
			printf("Third:\t%i\n", i);
			isThirdEmpty = false;
			break;
		}
	}
    }
	
    std_msgs::Float32 lwheel;
    std_msgs::Float32 rwheel;
	
    if(isSecondEmpty) {
	lwheel.data = 0.7;
	rwheel.data = 0.7;
    } else if(isFirstEmpty) {
	lwheel.data = 0.7;
	rwheel.data = -0.7;
    } else {
	lwheel.data = -0.7;
	rwheel.data = 0.7;
    }
	
    l_pub.publish(lwheel);
    r_pub.publish(rwheel);
    printf("LWheel:\t%f\n", lwheel.data);
    printf("RWheel:\t%f\n", rwheel.data);
    printf("Stop\n");

    r.sleep();
  }
}