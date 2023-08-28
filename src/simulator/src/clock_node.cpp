#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

class ClockPublisher
{
public:
  ClockPublisher()
  {
    publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);
    timer_ = nh_.createTimer(ros::Duration(0.01), &ClockPublisher::timerCallback, this);
    message_.clock = ros::Time(0);
    ROS_INFO("Virtual Clock has been created");
  }

private:
  void timerCallback(const ros::TimerEvent&)
  {
    message_.clock += ros::Duration(0.01);
    publisher_.publish(message_);
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher publisher_;
  rosgraph_msgs::Clock message_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clock_publisher");
  ClockPublisher clockPublisher;
  ros::spin();
  return 0;
}
