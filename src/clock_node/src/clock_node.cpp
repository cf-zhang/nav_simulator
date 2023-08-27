#include"clock_node.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "clock_publisher");
  ClockPublisher clockPublisher;
  ros::spin();
  return 0;
}