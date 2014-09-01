#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <trobot/Odometry.h>

#define KEYCODE_F 0x41
#define KEYCODE_B 0x42
#define KEYCODE_L 0x44
#define KEYCODE_R 0x43
#define KEYCODE_S 0x20

int kfd = 0;
struct termios cooked, raw;

class TeleopConsole
{
private:
  ros::NodeHandle nodeHandler;
  double left, right;
  ros::Publisher publisher;

public:
  TeleopConsole()
  {
      publisher = nodeHandler.advertise<trobot::Odometry>("trobot/teleop", 1);
  }

  void keyLoop()
  {
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the trobot.");


    for(;;)
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      right = left = 0;
      ROS_DEBUG("value: 0x%02X\n", c);
  
      switch(c)
      {
        case KEYCODE_L:
          ROS_DEBUG("left");
          left = -350.0;
	  right = 350.0;
          dirty = true;
          break;
        case KEYCODE_R:
          ROS_DEBUG("right");
          right = -350.0;
	  left = 350.0;
          dirty = true;
          break;
	case KEYCODE_F:
          ROS_DEBUG("forward");
          left = 350.0;
          right = 350.0;
          dirty = true;
          break;
	case KEYCODE_B:
          ROS_DEBUG("backward");
	  left = -350.0;
          right = -350.0;
          dirty = true;
          break;
	case KEYCODE_S:
          ROS_DEBUG("stop");
	  left = 0.0;
          right = 0.0;
          dirty = true;
          break;
      }
   
      if(dirty == true)
      {
	trobot::Odometry teleop;
        teleop.leftWheelSpeed = left;
        teleop.rightWheelSpeed = right;

        publisher.publish(teleop);
        dirty=false;
      }
    }


    return;
  }
};

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_console");
  TeleopConsole teleop_console;

  signal(SIGINT,quit);

  teleop_console.keyLoop();
  
  return(0);
}
