#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

namespace simulator{

class TeleControl : public rclcpp::Node
{
public:
  TeleControl(const std::string& name);
  void keyLoop();

private:
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

TeleControl::TeleControl(const std::string& name):
  rclcpp::Node("telecontrol"),
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(name + "/cmd_vel", 1);
}


void TeleControl::keyLoop()
{
  char c;
  bool dirty=false;


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
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
  
    switch(c)
    {
      case KEYCODE_L:
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        linear_ = -1.0;
        dirty = true;
        break;
    }
   
    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(std::move(twist));    
      dirty=false;
    }
  }
  return;
}
}

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::string name = "";
  if(argc > 2){
    puts("usages: handlebar robot_name");
    return 0;
  }else if (argc == 2){
    name = std::string(argv[1]);
  }

  simulator::TeleControl teleop_turtle(name);
  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}



