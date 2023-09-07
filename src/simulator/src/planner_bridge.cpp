#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

namespace simulator{

class PlannerBridge: public rclcpp::Node
{
public:
PlannerBridge()
: rclcpp::Node("planner_bridge")
{
  // Parameter initialization
  declare_parameter("planner_id", rclcpp::ParameterValue("GridBased"));
  get_parameter("planner_id", planner_id_);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerBridge::onGoalReceived, this, std::placeholders::_1));

  point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerBridge::onPointReceived, this, std::placeholders::_1));

  pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      this,
      "compute_path_to_pose");

  poses_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(
      this,
      "compute_path_through_poses");
}

protected:

void onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  std::cout<<"123 "<<planner_id_<<std::endl;
  if(poses_.size() > 0){
    poses_.push_back(*pose);
    auto goal_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();
    
    goal_msg.goals = poses_;
    goal_msg.planner_id = planner_id_;

    if (!poses_client_->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server %s not available after waiting", "compute_path_through_pose");
      return;
    }    
    poses_client_->async_send_goal(goal_msg, send_goal_options);
  }else{
    auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    goal_msg.goal = *pose;
    goal_msg.planner_id = planner_id_;
    if (!pose_client_->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server %s not available after waiting", "compute_path_to_pose");
      return;
    }
    pose_client_->async_send_goal(goal_msg, send_goal_options);
  }
  poses_.clear();
}

void onPointReceived(const geometry_msgs::msg::PointStamped::SharedPtr pose)
{
  geometry_msgs::msg::PoseStamped p;
  p.header = pose->header;
  p.pose.position.x = pose->point.x;
  p.pose.position.y = pose->point.y;
  poses_.push_back(p);
}

private:

std::string planner_id_;
std::vector<geometry_msgs::msg::PoseStamped> poses_;

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;

rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr pose_client_;
rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr poses_client_;
};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto planner_bridge = std::make_shared<simulator::PlannerBridge>();
  rclcpp::spin(planner_bridge->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
