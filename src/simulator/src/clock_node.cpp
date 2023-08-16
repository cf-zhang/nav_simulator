#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;

namespace simulator{

class ClockPublisher : public rclcpp::Node
{
public:
ClockPublisher()
: rclcpp::Node("clock_publisher"),message_(rosgraph_msgs::msg::Clock())
{

  publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  timer_ = this->create_wall_timer(
    10ms, std::bind(&ClockPublisher::timer_callback, this));
}

private:

void timer_callback()
{
  message_.clock.nanosec += 10000000;
  if(message_.clock.nanosec >= 1000000000){
      message_.clock.sec += 1;
      message_.clock.nanosec = 0;
  }
  publisher_->publish(std::move(message_));
}
rclcpp::TimerBase::SharedPtr timer_;
rosgraph_msgs::msg::Clock message_;
rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
};
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto clock_node = std::make_shared<simulator::ClockPublisher>();
  rclcpp::spin(clock_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
